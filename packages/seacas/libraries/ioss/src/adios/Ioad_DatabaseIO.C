// Copyright(C) 1999-2010 National Technology & Engineering Solutions
// of Sandia, LLC (NTESS).  Under the terms of Contract DE-NA0003525 with
// NTESS, the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of NTESS nor the names of its
//       contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "Ioss_CommSet.h"         // for CommSet
#include "Ioss_DBUsage.h"         // for DatabaseUsage, etc
#include "Ioss_DatabaseIO.h"      // for DatabaseIO
#include "Ioss_EdgeBlock.h"       // for EdgeBlock
#include "Ioss_EdgeSet.h"         // for EdgeSet
#include "Ioss_ElementBlock.h"    // for ElementBlock
#include "Ioss_ElementSet.h"      // for ElementSet
#include "Ioss_EntityType.h"      // for EntityType::ELEMENTBLOCK
#include "Ioss_FaceBlock.h"       // for FaceBlock
#include "Ioss_FaceSet.h"         // for FaceSet
#include "Ioss_Field.h"           // for Field, etc
#include "Ioss_FileInfo.h"        // for FileInfo
#include "Ioss_GroupingEntity.h"  // for GroupingEntity
#include "Ioss_Map.h"             // for Map, MapContainer
#include "Ioss_NodeBlock.h"       // for NodeBlock
#include "Ioss_NodeSet.h"         // for NodeSet
#include "Ioss_Property.h"        // for Property
#include "Ioss_Region.h"          // for Region, SideSetContainer, etc
#include "Ioss_SideBlock.h"       // for SideBlock
#include "Ioss_SideSet.h"         // for SideBlockContainer, SideSet
#include "Ioss_VariableType.h"    // for VariableType
#include <Ioss_CodeTypes.h>       // for HAVE_MPI
#include <Ioss_ElementTopology.h> // for NameList
#include <Ioss_ParallelUtils.h>   // for ParallelUtils, etc
#include <Ioss_SerializeIO.h>     // for SerializeIO
#include <Ioss_Utils.h>           // for Utils, IOSS_ERROR, etc
//#include <algorithm>              // for fill_n, find, fill
//#include <assert.h>               // for assert
//#include <cstring>                // for nullptr, strncasecmp, strcpy, etc
//#include <ctype.h>                // for isdigit
//#include <iostream>               // for cout
//#include <map>                    // for _Rb_tree_iterator, etc
//#include <stdio.h>                // for remove
//#include <stdlib.h>               // for atoi
//#include <string>                 // for char_traits, string, etc
//#include <vector>                 // for vector, etc
#include <adios/Ioad_DatabaseIO.h>
namespace Ioss {
  class PropertyManager;
}

static const char *Version = "2018/06/22";

namespace Ioad {
  // ========================================================================
  DatabaseIO::DatabaseIO(Ioss::Region *region, const std::string &filename,
                         Ioss::DatabaseUsage db_usage, MPI_Comm communicator,
                         const Ioss::PropertyManager &properties_x)
      : Ioss::DatabaseIO(region, filename, db_usage, communicator, properties_x)
  {
    Ioss::SerializeIO serializeIO__(this);

    dbState = Ioss::STATE_UNKNOWN;

    //    // Construct the HDF and XML filenames for XDMF
    //    std::string decoded_name = util().decode_filename(filename, isParallel);

    //    hdfname = Ioss::FileInfo(decoded_name + ".h5");
    //    xmlname = Ioss::FileInfo(decoded_name + ".xmf");

    ad = new adios2::ADIOS(communicator);
    //    adios2::ADIOS adios(communicator);
    adios2::IO bpio = ad->DeclareIO("writer");

    // adios2::IO bpio = ad->DeclareIO("writer");
    // bpiop = &bpio;
    // if not defined by user, we can change the default settings
    // BPFile is the default engine
    bpio.SetEngine("BPFile");
    // bpio->SetParameters({{"num_threads", "1"}});

    // ISO-POSIX file output is the default transport (called "File")
    // Passing parameters to the transport
    bpio.AddTransport("File", {{"Library", "POSIX"}});
    bpio.DefineAttribute("IOSS_adios_version", 1);

    bpWriter = bpio.Open(filename, adios2::Mode::Write, communicator);
  }

  DatabaseIO::~DatabaseIO()
  {
    // bpWriter->Close();
    delete ad;
  }

  bool DatabaseIO::begin__(Ioss::State state)
  {
    dbState = state;
    if(state == Ioss::STATE_MODEL || state == Ioss::STATE_DEFINE_TRANSIENT) {
      bpWriter.BeginStep();
    }
    return true;
  }

  bool DatabaseIO::end__(Ioss::State state)
  {
    // Transitioning out of state 'state'
    assert(state == dbState);
    switch (state) {
    case Ioss::STATE_DEFINE_MODEL:
      if (!is_input())
        write_meta_data();
      break;
    case Ioss::STATE_MODEL:
       if (!is_input()) {
         bpWriter.EndStep();
       }
      break;
    case Ioss::STATE_DEFINE_TRANSIENT:
      if (!is_input())
            bpWriter.EndStep();
        //      write_results_metadata();
        break;
    default: // ignore everything else...
      if (!is_input()) {
        //        WriteXmlFile(NumOfIterations);
      }
      break;
    }

    {
      Ioss::SerializeIO serializeIO__(this);
      dbState = Ioss::STATE_UNKNOWN;
    }

    return true;
  }

  template <typename T>
  void DatabaseIO::write_meta_data_internal(adios2::IO &bpio, const Ioss::Field &field,
                                            const std::string &encoded_name) const
  {
    static int rank        = Ioss::SerializeIO::getRank();
    static int number_proc = Ioss::SerializeIO::getSize();
    const Ioss::VariableType *field_var = field.raw_storage(); 
    size_t     local_size  = field.raw_count() * field_var->component_count();
    //size_t global_size     = number_proc * local_size;
    bpio.DefineVariable<T>(encoded_name, {number_proc, INT_MAX}, {rank, 0}, {1, local_size});
  }

  void DatabaseIO::write_meta_data()
  {
    adios2::IO bpio = ad->AtIO("writer");

    Ioss::Region *                  region      = get_region();
    const Ioss::NodeBlockContainer &node_blocks = region->get_node_blocks();

    // A single nodeblock named "nodeblock_1" will be created for the mesh. It contains information
    // for every node that exists in the model (Ioss-exodus-mapping.pdf).
    assert(node_blocks.size() == 1);
    //int spatialDimension = node_blocks[0]->get_property("component_degree").get_int();

    Ioss::NameList names;
    // Even though the region should contain only one nodeblock, we keep this code compatible with
    // potentially multiple nodeblocks, just for future proof purpose.
    for (auto node_block : node_blocks) {
      std::string block_name = node_blocks[0]->type_string();

      node_block->field_describe(&names);
      for (auto name : names) {
        // Allocate Nodeblock
        auto        field        = node_block->get_fieldref(name);
        std::string encoded_name = block_name + "-" + name;
        entityNames[std::make_pair(name, node_blocks[0]->type_string())] = encoded_name;
        switch (field.get_type()) {
        case Ioss::Field::BasicType::DOUBLE:
          write_meta_data_internal<double>(bpio, field, encoded_name);
          break;
        case Ioss::Field::BasicType::INT32:
          write_meta_data_internal<int32_t>(bpio, field, encoded_name);
          break;
        case Ioss::Field::BasicType::INT64:
          write_meta_data_internal<int64_t>(bpio, field, encoded_name);
          break;
        case Ioss::Field::BasicType::COMPLEX:
          write_meta_data_internal<Complex>(bpio, field, encoded_name);
          break;
        case Ioss::Field::BasicType::CHARACTER:
          write_meta_data_internal<char>(bpio, field, encoded_name);
          break;
        case Ioss::Field::BasicType::STRING:
          bpio.DefineVariable<std::string>(
              encoded_name,
              // Global dimensions
              {field.raw_count()},
              // starting offset of the local array in the global space
              {0},
              // local size, could be defined later using SetSelection()
              {field.raw_count()});
          break;
        default:
          std::ostringstream errmsg;
          errmsg << "INTERNAL ERROR: Invalid field type. "
                 << "Something is wrong in the Ioad::DatabaseIO::write_meta_data() function. "
                 << "Please report.\n";
          IOSS_ERROR(errmsg);
        }
      }
    }
  }

  //------------------------------------------------------------------------
  int64_t DatabaseIO::put_field_internal(const Ioss::Region * /* region */,
                                         const Ioss::Field &field, void *data,
                                         size_t data_size) const
  {
    {
      Ioss::SerializeIO serializeIO__(this);

      Ioss::Field::RoleType role       = field.get_role();
      int                   num_to_get = field.verify(data_size);

      return num_to_get;
    }
  }

  template <typename T>
  void DatabaseIO::put_data(adios2::IO &bpio, const Ioss::Field &field, void *data,
                            const std::string &encoded_name) const
  {
    adios2::Variable<T> entities = bpio.InquireVariable<T>(encoded_name);
    if (entities) {
      T *rdata = static_cast<T *>(data);
      bpWriter.Put<T>(entities, rdata, adios2::Mode::Sync); // If not Sync, variables are not saved correctly.
    }
  }

  // TODO: write actual code!
  // Returns byte size of integers stored on the database...
  int DatabaseIO::int_byte_size_db() const { return 4; }

  int64_t DatabaseIO::put_field_internal(const Ioss::NodeBlock *nb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    size_t num_to_get = field.verify(data_size); // Compare to size reserved in io variable instead?
    Ioss::Field::RoleType role = field.get_role();

    size_t proc_offset = 0;
    if (nb->property_exists("processor_offset")) {
      proc_offset = nb->get_property("processor_offset").get_int();
    }
    size_t file_count = num_to_get;
    if (nb->property_exists("locally_owned_count")) {
      file_count = nb->get_property("locally_owned_count").get_int();
    }

    if (role == Ioss::Field::MESH) {
      if (field.get_name() == "owning_processor") {
      }

      else if (field.get_name() == "mesh_model_coordinates_x") {
      }
      else if (field.get_name() == "mesh_model_coordinates_y") {
      }
      else if (field.get_name() == "mesh_model_coordinates_z") {
      }
      else if (field.get_name() == "mesh_model_coordinates") {
      }
      else if (field.get_name() == "ids") {
        // The ids coming in are the global ids; their position is the
        // local id -1 (That is, data[0] contains the global id of local
        // node 1)

        // Another 'const-cast' since we are modifying the database just
        // for efficiency; which the client does not see...
        // handle_node_ids(data, num_to_get, proc_offset, file_count);
      }
      else if (field.get_name() == "connectivity") {
        // Do nothing, just handles an idiosyncrasy of the GroupingEntity
      }
      else if (field.get_name() == "connectivity_raw") {
        // Do nothing, just handles an idiosyncrasy of the GroupingEntity
      }
      else if (field.get_name() == "node_connectivity_status") {
        // Do nothing, input only field.
      }
      else if (field.get_name() == "implicit_ids") {
        // Do nothing, input only field.
      }
      else {
      }
    }
    else if (role == Ioss::Field::TRANSIENT) {
    }
    else if (role == Ioss::Field::REDUCTION) {
    }

    adios2::IO bpio = ad->AtIO("writer");
    // for (auto const& pairName : entityNames) {
    std::string encoded_name =
        entityNames.at(std::make_pair(field.get_name(), nb[0].type_string()));
std::cout<<encoded_name<<std::endl;
    switch (field.get_type()) {
    case Ioss::Field::BasicType::DOUBLE: put_data<double>(bpio, field, data, encoded_name); break;
    case Ioss::Field::BasicType::INT32: put_data<int32_t>(bpio, field, data, encoded_name); break;
    case Ioss::Field::BasicType::INT64: put_data<int64_t>(bpio, field, data, encoded_name); break;
    case Ioss::Field::BasicType::COMPLEX: put_data<Complex>(bpio, field, data, encoded_name); break;
    case Ioss::Field::BasicType::CHARACTER: put_data<char>(bpio, field, data, encoded_name); break;
    case Ioss::Field::BasicType::STRING:
      put_data<std::string>(bpio, field, data, encoded_name);
      break;
    default:
      std::ostringstream errmsg;
      errmsg << "INTERNAL ERROR: Invalid field type. "
             << "Something is wrong in the Ioad::DatabaseIO::put_field_internal() function. "
             << "Please report.\n";
      IOSS_ERROR(errmsg);
    }
  }

} // namespace Ioad

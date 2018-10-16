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
#include "adios2/helper/adiosFunctions.h"
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
    rank        = Ioss::SerializeIO::getRank();
    number_proc = Ioss::SerializeIO::getSize();
    ad      = new adios2::ADIOS(communicator);
    dbState = Ioss::STATE_UNKNOWN;
    if (!is_input()) {
      adios2::IO bpio = ad->DeclareIO("writer");
      bpio.SetEngine("BPFile");
      bpio.AddTransport("File", {{"Library", "POSIX"}});
      bp_engine = bpio.Open(filename, adios2::Mode::Write, communicator);
    }
    else {
      adios2::IO reader_io = ad->DeclareIO("reader");
      reader_io.SetEngine("BPFile");
      reader_io.AddTransport("File", {{"Library", "POSIX"}});
      bp_engine = reader_io.Open(filename, adios2::Mode::Read, communicator);
    }
  }

  DatabaseIO::~DatabaseIO()
  {
    bp_engine.Close();
    delete ad;
  }

  bool DatabaseIO::begin__(Ioss::State state)
  {
    dbState = state;
    if (state == Ioss::STATE_MODEL || state == Ioss::STATE_DEFINE_TRANSIENT) {
      bp_engine.BeginStep();
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
        define_model();
      break;
    case Ioss::STATE_MODEL:
      if (!is_input()) {
        bp_engine.EndStep();
      }
      break;
    case Ioss::STATE_DEFINE_TRANSIENT:
      if (!is_input())
        bp_engine.EndStep();
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
  void DatabaseIO::define_model_internal(adios2::IO &bpio, const Ioss::Field &field,
                                         const std::string &encoded_name)
  {
    const Ioss::VariableType *field_var = field.raw_storage();
    int component_count = field_var->component_count();
    size_t local_size = field.raw_count();
    bpio.DefineVariable<T>(encoded_name, {number_proc, INT_MAX, component_count}, {rank, 0, 0}, {1, local_size, component_count});
  }

  template <typename T>
  void DatabaseIO::define_entity_internal(const T &entity_blocks, adios2::IO &bpio)
  {
    Ioss::NameList names;
    for (auto entity_block : entity_blocks) {
      std::string entity_name = entity_blocks[0]->type_string();

      entity_block->field_describe(&names);
      for (auto name : names) {
        // Define entity block variables
        auto        field        = entity_block->get_fieldref(name);
        std::string encoded_name = entity_name + "/" + name;
        entityNames[std::make_pair(name, entity_block->type_string())] = encoded_name;
        switch (field.get_type()) {
        case Ioss::Field::BasicType::DOUBLE:
          define_model_internal<double>(bpio, field, encoded_name);
          break;
        case Ioss::Field::BasicType::INT32:
          define_model_internal<int32_t>(bpio, field, encoded_name);
          break;
        case Ioss::Field::BasicType::INT64:
          define_model_internal<int64_t>(bpio, field, encoded_name);
          break;
        case Ioss::Field::BasicType::COMPLEX:
          define_model_internal<Complex>(bpio, field, encoded_name);
          break;
        case Ioss::Field::BasicType::CHARACTER:
          define_model_internal<char>(bpio, field, encoded_name);
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
          errmsg
              << "INTERNAL ERROR: Invalid field type. "
              << "Something is wrong in the Ioad::DatabaseIO::define_entity_internal() function. "
              << "Please report.\n";
          IOSS_ERROR(errmsg);
        }
      }
    }
  }

  // Similar to `write_meta_data()` function in other DatabaseIO. This function has been renamed in
  // this database to reflect more precisely what it accomplishes.
  void DatabaseIO::define_model()
  {

    Ioss::Region *                  region      = get_region();
    const Ioss::NodeBlockContainer &node_blocks = region->get_node_blocks();

    // A single nodeblock named "nodeblock_1" will be created for the mesh. It contains information
    // for every node that exists in the model (Ioss-exodus-mapping.pdf).
    assert(node_blocks.size() == 1);
    // int spatialDimension = node_blocks[0]->get_property("component_degree").get_int();

    adios2::IO bpio = ad->AtIO("writer");

    bpio.DefineAttribute<unsigned int>(schema_version_string, 1);

    define_entity_internal<Ioss::NodeBlockContainer>(node_blocks, bpio);
    // Edge Blocks --
    const Ioss::EdgeBlockContainer &edge_blocks = region->get_edge_blocks();
    define_entity_internal<Ioss::EdgeBlockContainer>(edge_blocks, bpio);
    // Face Blocks --
    const Ioss::FaceBlockContainer &face_blocks = region->get_face_blocks();
    define_entity_internal<Ioss::FaceBlockContainer>(face_blocks, bpio);
    // Element Blocks --
    const Ioss::ElementBlockContainer &element_blocks = region->get_element_blocks();
    define_entity_internal<Ioss::ElementBlockContainer>(element_blocks, bpio);
    // Nodesets ...
    const Ioss::NodeSetContainer &nodesets = region->get_nodesets();
    define_entity_internal<Ioss::NodeSetContainer>(nodesets, bpio);
    // Edgesets ...
    const Ioss::EdgeSetContainer &edgesets = region->get_edgesets();
    define_entity_internal<Ioss::EdgeSetContainer>(edgesets, bpio);
    // Facesets ...
    const Ioss::FaceSetContainer &facesets = region->get_facesets();
    define_entity_internal<Ioss::FaceSetContainer>(facesets, bpio);
    // Elementsets ...
    const Ioss::ElementSetContainer &elementsets = region->get_elementsets();
    define_entity_internal<Ioss::ElementSetContainer>(elementsets, bpio);
    // SideSets ...
    // const Ioss::SideSetContainer &ssets = region->get_sidesets();
    // define_entity_internal<Ioss::SideSetContainer>(ssets);
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
      bp_engine.Put<T>(entities, rdata,
                      adios2::Mode::Sync); // If not Sync, variables are not saved correctly.
    }
  }

  // TODO: write actual code!
  // Returns byte size of integers stored on the database...
  int DatabaseIO::int_byte_size_db() const { return 4; }

  int64_t DatabaseIO::put_field_internal(const Ioss::NodeBlock *nb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    put_field_internal(nb->type_string(), field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const std::string &type_string, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    size_t num_to_get = field.verify(data_size); // Compare to size reserved in io variable instead?
    Ioss::Field::RoleType role = field.get_role();

    adios2::IO bpio = ad->AtIO("writer");
    // for (auto const& pairName : entityNames) {
    std::string encoded_name = entityNames.at(std::make_pair(field.get_name(), type_string));

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

  int64_t DatabaseIO::put_field_internal(const Ioss::ElementBlock *eb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    put_field_internal(eb->type_string(), field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::EdgeBlock *eb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    put_field_internal(eb->type_string(), field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::FaceBlock *fb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    put_field_internal(fb->type_string(), field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::SideBlock *sb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    put_field_internal(sb->type_string(), field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::NodeSet *ns, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    put_field_internal(ns->type_string(), field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::EdgeSet *es, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    put_field_internal(es->type_string(), field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::FaceSet *fs, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    put_field_internal(fs->type_string(), field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::ElementSet *es, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    put_field_internal(es->type_string(), field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::SideSet *ss, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    put_field_internal(ss->type_string(), field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::CommSet *cs, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    put_field_internal(cs->type_string(), field, data, data_size);
  }

  template <typename T>
  void DatabaseIO::read_variable_size(adios2::IO &bpio, std::pair<std::string, std::map<std::string, std::string>> vpair)
  {
    auto v = bpio.InquireVariable<T>(vpair.first);
     std::map<size_t, std::vector<typename adios2::Variable<T>::Info>> allblocks = bp_engine.AllStepsBlocksInfo(v);
    if (!allblocks.empty()) {
      size_t ndim = v.Shape().size();
      /*Only query the block size of the current process based on the process rank.*/
      if (ndim == 0) {
        exit(0); /*not handled. Crash to not forget that it has to be improved.*/
      }
      else {
        size_t laststep = allblocks.rbegin()->first;
        size_t ndim     = v.Shape().size();
        for (auto &blockpair : allblocks) {
          size_t                                  step   = blockpair.first;
          std::vector<typename adios2::Variable<T>::Info> &blocks = blockpair.second;
          std::cout << "step: " << step << std::endl;
          std::cout << "block:" << std::endl;
          for (size_t j = 0; j < blocks.size(); j++) {
            for (size_t k = 0; k < ndim; k++) {
              if (blocks[j].Count[k]) {
                std::cout << "!" << blocks[j].Start[k] << ":"
                          << blocks[j].Start[k] + blocks[j].Count[k] - 1;
              }
              else {
                exit(0); /*Not handled*/
              }
              if (k < ndim - 1)
                std::cout << " - ";
            }
            std::cout << std::endl;
          }
        }
      }
    }
  }

  void DatabaseIO::read_region(adios2::IO &bpio)
  {

    // Only get schema version attribute as it is the only one we expect.
    auto schema_version = bpio.InquireAttribute<unsigned int>(schema_version_string);
    if (!schema_version) {
      std::ostringstream errmsg;
      errmsg << "INTERNAL ERROR: schema_version_string not found. "
             << "Something is wrong in the Ioad::DatabaseIO::read_region() function. "
             << "Please check input file.\n";
      IOSS_ERROR(errmsg);
    }
    // Fow now, we do not do anything special based on the schema version. It is only used to check
    // that the input file is in the expected format.

    const std::map<std::string, std::map<std::string, std::string>> variables = bpio.AvailableVariables();

    for (const auto &vpair : variables) {
      const std::string &name = vpair.first;
      std::cout << "name:" << name << std::endl;
      // for (const auto &key : vpair.second) {
      // std::cout << "key:" << key.first << std::endl;
      // }
      std::cout << "type:" << vpair.second.at("Type") << std::endl;
      std::cout << "shape:" << vpair.second.at("Shape") << std::endl;
      std::cout << "step counts:" << vpair.second.at("AvailableStepsCount") << std::endl;

      // Simply to start the "else if" section with "if".
      if (vpair.second.at("Type") == "not supported") {
      }
#define declare_template_instantiation(T)                             \
  else if (vpair.second.at("Type") == adios2::helper::GetType<T>()) { \
      /*read_variable_size<T>(bpio, vpair);*/                                           \
      }
      ADIOS2_FOREACH_TYPE_1ARG(declare_template_instantiation)
#undef declare_template_instantiation

    // Once everything is loaded, get global variables such as spatialDimension.
    }
  }

  void DatabaseIO::read_meta_data__()
  {
    adios2::IO reader_io = ad->AtIO("reader");
    read_region(reader_io);
    // read_communication_metadata();

    // get_step_times__();

    // get_nodeblocks();
    // get_edgeblocks();
    // get_faceblocks();
    // get_elemblocks();

    // check_side_topology();

    // get_sidesets();
    // get_nodesets();
    // get_edgesets();
    // get_facesets();
    // get_elemsets();

    // get_commsets();

    // handle_groups();

    // add_region_fields();

    // if (!is_input() && open_create_behavior() == Ioss::DB_APPEND) {
    //   get_map(EX_NODE_BLOCK);
    //   get_map(EX_EDGE_BLOCK);
    //   get_map(EX_FACE_BLOCK);
    //   get_map(EX_ELEM_BLOCK);
    // }
  }

  // int64_t DatabaseIO::put_field_internal(const Ioss::StructuredBlock *sb, const Ioss::Field
  // &field,
  //                                        void *data, size_t data_size) const
  // {
  //   put_field_internal(sb->type_string(), field, data, data_size);
  // }

} // namespace Ioad

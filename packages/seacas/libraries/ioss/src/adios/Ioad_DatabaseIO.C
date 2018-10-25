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

namespace {
  template <typename T> Ioss::Field::BasicType template_to_basic_type()
  {
    return Ioss::Field::BasicType::INVALID;
  }

  template <> Ioss::Field::BasicType template_to_basic_type<double>()
  {
    return Ioss::Field::BasicType::DOUBLE;
  }

  template <> Ioss::Field::BasicType template_to_basic_type<int32_t>()
  {
    return Ioss::Field::BasicType::INT32;
  }

  template <> Ioss::Field::BasicType template_to_basic_type<int64_t>()
  {
    return Ioss::Field::BasicType::INT64;
  }

  template <> Ioss::Field::BasicType template_to_basic_type<Complex>()
  {
    return Ioss::Field::BasicType::COMPLEX;
  }

  template <> Ioss::Field::BasicType template_to_basic_type<std::string>()
  {
    return Ioss::Field::BasicType::STRING;
  }

  template <> Ioss::Field::BasicType template_to_basic_type<char>()
  {
    return Ioss::Field::BasicType::CHARACTER;
  }
} // namespace

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
    ad          = new adios2::ADIOS(communicator);
    dbState     = Ioss::STATE_UNKNOWN;
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
    // Always 64 bits
    dbIntSizeAPI = Ioss::USE_INT64_API;
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

  bool DatabaseIO::use_transformed_storage(const Ioss::Field &field, const std::string &entity_type,
                                           const std::string &field_name) const
  {
    if (field.get_role() == Ioss::Field::RoleType::TRANSIENT ||
        find_field_in_mapset(entity_type, field_name, use_transformed_storage_map)) {
      return true;
    }
    return false;
  }

  template <typename T>
  void DatabaseIO::define_model_internal(adios2::IO &bpio, const Ioss::Field &field,
                                         const std::string &encoded_name)
  {
    size_t      component_count;
    size_t      local_size;
    std::string entity_type, field_name;
    std::tie(entity_type, std::ignore, field_name) = decode_field_name(encoded_name);
    if (find_field_in_mapset(entity_type, field_name, ignore_fields)) {
      return;
    }
    if (use_transformed_storage(field, entity_type, field_name)) {
      component_count = field.transformed_storage()->component_count();
      local_size      = field.transformed_count();
    }
    else {
      component_count = field.raw_storage()->component_count();
      local_size      = field.raw_count();
    }
    std::cout << "define: " << encoded_name << std::endl;
    bpio.DefineVariable<T>(encoded_name, {number_proc, INT_MAX, component_count}, {rank, 0, 0},
                           {1, local_size, component_count});
  }

  std::string DatabaseIO::encode_field_name(const std::string &entity_type,
                                            const std::string &entity_name,
                                            const std::string &field_name) const
  {
    return entity_type + name_separator + entity_name + name_separator + field_name;
  }

  std::tuple<std::string, std::string, std::string>
  DatabaseIO::decode_field_name(const std::string &encoded_name) const
  {
    std::size_t current, previous = 0;
    current = encoded_name.find_first_of(name_separator);
    std::vector<std::string> container;
    // Stop after finding `name_separator` twice as the result is suppose to be
    // a triplet `entity_type`/`entity_name`/`field_name`
    // This means `field_name` is allowed to contain `name_separator`.
    while (current != std::string::npos && container.size() <= 2) {
      container.push_back(encoded_name.substr(previous, current - previous));
      previous = current + 1;
      current  = encoded_name.find_first_of(name_separator, previous);
    }
    container.push_back(encoded_name.substr(previous, current - previous));
    if (container.size() != 3) {
      std::ostringstream errmsg;
      errmsg << "ERROR: Invalid encoded entity name. Does not contain 2 separators.\n";
      IOSS_ERROR(errmsg);
    }
    return make_tuple(container[0], container[1], container[2]);
  }

  template <typename T>
  void DatabaseIO::define_entity_internal(const T &entity_blocks, adios2::IO &bpio)
  {
    Ioss::NameList field_names;
    for (auto entity_block : entity_blocks) {
      std::string entity_type = entity_blocks[0]->type_string();
      std::string entity_name = entity_blocks[0]->name();
      entity_block->field_describe(&field_names);
      for (auto field_name : field_names) {
        // Define entity block variables
        auto        field        = entity_block->get_fieldref(field_name);
        std::string encoded_name = encode_field_name(entity_type, entity_name, field_name);
        switch (field.get_type()) {
        // REAL and DOUBLE have same value. Code is left here but commented out for documentation
        // purposes.
        // case Ioss::Field::BasicType::REAL:
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
    const Ioss::SideSetContainer &ssets = region->get_sidesets();
    define_entity_internal<Ioss::SideSetContainer>(ssets, bpio);
    // SideBlocks ...
    for (auto &sset : ssets) {
      // Until  local strings or GlobalArray of strings are supported, use multiple GlobalValue
      // variables, one per process (rank), to avoid having to create a char GlobalArray with extra
      // dimensions and fixed size components. In practice, the name of the side_blocks are unlikely
      // to change during computation, but this implementation allows us to not have to make any
      // assumption.
      std::string encoded_name                = encode_field_name(sset->type_string(), sset->name(),
                                                   "sideblock_names_" + std::to_string(rank));
      const Ioss::SideBlockContainer &sblocks = sset->get_side_blocks();
      // bpio.DefineVariable<std::string>(encoded_name, {number_proc, INT_MAX, 1}, {rank, 0, 0},
      //                     {1, sblocks.size(), 1});
      bpio.DefineVariable<std::string>(encoded_name);
      define_entity_internal<Ioss::SideBlockContainer>(sblocks, bpio);
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
  int64_t DatabaseIO::put_data(adios2::IO &bpio, const Ioss::Field &field, void *data,
                               const std::string &encoded_name, bool transformed_field,
                               size_t data_size) const
  {
    adios2::Variable<T> entities   = bpio.InquireVariable<T>(encoded_name);
    int                 num_to_get = field.verify(data_size);

    if (entities && data && data_size) {

      std::cout << "Put:" << encoded_name << std::endl;
      T *rdata = static_cast<T *>(data);
      bp_engine.Put<T>(entities, rdata,
                       adios2::Mode::Sync); // If not Sync, variables are not saved correctly.

      // Defining role attribute only when calling "Put" to avoid creating attributes for
      // fields that are not saved.
      bpio.DefineAttribute<int>(role_attribute, field.get_role(), encoded_name,
                                attribute_separator);
      if (transformed_field) {
        bpio.DefineAttribute<std::string>(var_type_attribute, field.transformed_storage()->name(),
                                          encoded_name, attribute_separator);
      }
      else {
        bpio.DefineAttribute<std::string>(var_type_attribute, field.raw_storage()->name(),
                                          encoded_name, attribute_separator);
      }
      return num_to_get;
    }
    return 0;
  }

  // TODO: write actual code!
  // Returns byte size of integers stored on the database...
  int DatabaseIO::int_byte_size_db() const { return 4; }

  int64_t DatabaseIO::put_field_internal(const Ioss::NodeBlock *nb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return put_field_internal(nb->type_string(), nb->name(), field, data, data_size);
  }

  int DatabaseIO::find_field_in_mapset(
      const std::string &entity_type, const std::string &field_name,
      const std::map<std::string, std::set<std::string>> &mapset) const
  {
    if (mapset.find(entity_type) == mapset.end()) {
      // No field for this entity_type in the map.
      return 0;
    }
    const std::set<std::string> &entity_set = mapset.at(entity_type);
    if (entity_set.find(field_name) != entity_set.end()) {
      return 1;
    }
    return 0;
  }

  int64_t DatabaseIO::put_field_internal(const std::string &entity_type,
                                         const std::string &entity_name, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    const std::string &field_name = field.get_name();
    if (find_field_in_mapset(entity_type, field_name, ignore_fields)) {
      return 0;
    }

    adios2::IO  bpio              = ad->AtIO("writer");
    std::string encoded_name      = encode_field_name(entity_type, entity_name, field_name);
    bool        transformed_field = use_transformed_storage(field, entity_type, field_name);

    switch (field.get_type()) {
    case Ioss::Field::BasicType::DOUBLE:
      return put_data<double>(bpio, field, data, encoded_name, transformed_field, data_size);
      break;
    case Ioss::Field::BasicType::INT32:
      return put_data<int32_t>(bpio, field, data, encoded_name, transformed_field, data_size);
      break;
    case Ioss::Field::BasicType::INT64:
      return put_data<int64_t>(bpio, field, data, encoded_name, transformed_field, data_size);
      break;
    case Ioss::Field::BasicType::COMPLEX:
      return put_data<Complex>(bpio, field, data, encoded_name, transformed_field, data_size);
      break;
    case Ioss::Field::BasicType::CHARACTER:
      return put_data<char>(bpio, field, data, encoded_name, transformed_field, data_size);
      break;
    default:
      std::ostringstream errmsg;
      errmsg << "INTERNAL ERROR: Invalid field type. "
             << "Something is wrong in the Ioad::DatabaseIO::put_field_internal() function. "
             << "Please report.\n";
      IOSS_ERROR(errmsg);
    }
    return 0;
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::ElementBlock *eb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return put_field_internal(eb->type_string(), eb->name(), field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::EdgeBlock *eb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return put_field_internal(eb->type_string(), eb->name(), field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::FaceBlock *fb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return put_field_internal(fb->type_string(), fb->name(), field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::SideBlock *sb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return put_field_internal(sb->type_string(), sb->name(), field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::NodeSet *ns, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return put_field_internal(ns->type_string(), ns->name(), field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::EdgeSet *es, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return put_field_internal(es->type_string(), es->name(), field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::FaceSet *fs, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return put_field_internal(fs->type_string(), fs->name(), field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::ElementSet *es, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return put_field_internal(es->type_string(), es->name(), field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::SideSet *ss, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    int64_t res = put_field_internal(ss->type_string(), ss->name(), field, data, data_size);
    if (res) {
      adios2::IO                    bpio         = ad->AtIO("writer");
      std::string                   encoded_name = encode_field_name(ss->type_string(), ss->name(),
                                                   "sideblock_names" + std::to_string(rank));
      adios2::Variable<std::string> entities     = bpio.InquireVariable<std::string>(encoded_name);
      std::vector<std::string>      block_members;
      for (auto &sb : ss->get_side_blocks()) {
        block_members.push_back(sb->name());
      }
      if (entities && block_members.size()) {
        std::string stringified_block_members;
        for (std::string s : block_members) {
          stringified_block_members += "/" + s;
        }
        bp_engine.Put<std::string>(
            entities, stringified_block_members,
            adios2::Mode::Sync); // If not Sync, variables are not saved correctly.
      }
    }
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::CommSet *cs, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return put_field_internal(cs->type_string(), cs->name(), field, data, data_size);
  }

  // template<typename T>
  // create_nodeblock(std::string block_name, std::vector<typename adios2::Variable<T>::Info>
  // blocks)
  // {
  // // Check number of nodeblocks/ids==1
  // // Create nodeblock
  // // Load data
  // // Load other nodeblock fields
  // }

  // void DatabaseIO::add_attribute_fields(
  //     Ioss::GroupingEntity *block, std::vector<std::pair<size_t, size_t>> size,
  //     std::map<std::string, std::pair<std::string, std::string>> field_names)
  // {
  //   // Does not verify if name is correct based on IOSS requirements. Assumes that it is the
  //   case. size_t my_element_count = block->entity_count();

  //   // std::vector<Ioss::Field> attributes;
  //   // Ioss::Utils::get_fields(my_element_count, field_names, Ioss::Field::ATTRIBUTE,
  //   //                             get_field_recognition(), field_suffix_separator, nullptr,
  //   //                             attributes);
  //   //     int offset = 1;
  //   //     for (const auto &field : attributes) {
  //   //       if (block->field_exists(field.get_name())) {
  //   //         std::ostringstream errmsg;
  //   //         errmsg << "ERROR: In block '" << block->name() << "', attribute '" <<
  //   //         field.get_name()
  //   //                << "' is defined multiple times which is not allowed.\n";
  //   //         IOSS_ERROR(errmsg);
  //   //       }
  //   //       block->field_add(field);
  //   //       const Ioss::Field &tmp_field = block->get_fieldref(field.get_name());
  //   //       tmp_field.set_index(offset);
  //   //       offset += field.raw_storage()->component_count();
  //   //     }
  // }
  template <typename T>
  BlockInfoType DatabaseIO::get_variable_infos_from_map(adios2::IO &         bpio,
                                                        const EntityMapType &entity_map,
                                                        const std::string &  entity_name,
                                                        const std::string &  block_name,
                                                        const std::string &  var_name) const
  {
    // No check to verify that the block_name, entity_name, variable_name, and type exist as they
    // have been extracted earlier from the file and everything should still be up to date.
    if (entity_map.find(block_name) == entity_map.end()) {
      std::ostringstream errmsg;
      errmsg << "ERROR: block name " << block_name << " not found.\n";
      IOSS_ERROR(errmsg);
    }

    if (entity_map.at(block_name).find(var_name) == entity_map.at(block_name).end() ||
        entity_map.at(block_name).at(var_name).second != adios2::helper::GetType<T>()) {
      std::ostringstream errmsg;
      errmsg << "ERROR: block name " << block_name << " not found or does not contain `" << var_name
             << "` field.\n";
      IOSS_ERROR(errmsg);
    }
    return get_variable_infos_from_map_no_check(bpio, entity_map, entity_name, block_name,
                                                var_name);
  }

  BlockInfoType DatabaseIO::get_variable_infos_from_map_no_check(adios2::IO &         bpio,
                                                                 const EntityMapType &entity_map,
                                                                 const std::string &  entity_name,
                                                                 const std::string &  block_name,
                                                                 const std::string &var_name) const
  {
    std::string variable = encode_field_name(entity_name, block_name, var_name);
    std::string type     = entity_map.at(block_name).at(var_name).second;
    // Simply to start the "else if" section with "if".
    if (type == "not supported") {
    }
#define declare_template_instantiation(T)                                                          \
  else if (type == adios2::helper::GetType<T>()) { return get_variable_infos<T>(bpio, variable); }
    ADIOS2_FOREACH_TYPE_1ARG(declare_template_instantiation)
#undef declare_template_instantiation
    else
    {
      std::ostringstream errmsg;
      errmsg << "INTERNAL ERROR: Invalid variable type. "
             << "Something is wrong in the "
                "Ioad::DatabaseIO::get_variable_infos_from_map_no_check() function. "
             << "Please report.\n";
      IOSS_ERROR(errmsg);
    }
    return BlockInfoType();
  }

  // common
  void DatabaseIO::get_nodeblocks(adios2::IO &bpio, const VariableMapType &variables_map)
  {
    // For exodusII, there is only a single node block which contains
    // all of the nodes.
    // The default id assigned is '1' and the name is 'nodeblock_1'
    std::string entity_name = "NodeBlock";
    const std::map<std::string, std::map<std::string, std::pair<std::string, std::string>>>
        &       entity_map = variables_map.at(entity_name);
    std::string block_name = "nodeblock_1";

    // `mesh_model_coordinates` field is automatically created in NodeBlock constructor.
    std::string coord_var_name = "mesh_model_coordinates";
    // Get `node_count` and `spatialDimension`.
    BlockInfoType model_coordinates_infos = get_variable_infos_from_map<double>(
        bpio, entity_map, entity_name, block_name, coord_var_name);
    spatialDimension = model_coordinates_infos.component_count;
    if (!spatialDimension) {
      std::ostringstream errmsg;
      errmsg << "ERROR: Variable `" << coord_var_name
             << "` in BP file without any dimension information.\n";
      IOSS_ERROR(errmsg);
    }
    auto block = new Ioss::NodeBlock(this, block_name, model_coordinates_infos.node_boundaries_size,
                                     spatialDimension);

    Ioss::NameList field_names;
    block->field_describe(&field_names);
    for (std::string &s : field_names) {
      std::cout << "default fields:" << s << std::endl;
    }
    block->property_add(Ioss::Property("id", 1));
    block->property_add(Ioss::Property("guid", util().generate_guid(1)));
    std::cout << "reader:" << std::endl;
    for (auto &variable : entity_map.at(block_name)) {
      // Since some fields are created automatically, we need to avoid recreating them when loading
      // the file.
      if (!block->field_exists(variable.first)) {
        BlockInfoType variable_infos = get_variable_infos_from_map_no_check(
            bpio, entity_map, entity_name, block_name, variable.first);
        Ioss::Field field(variable.first, variable_infos.basic_type, variable_infos.variable_type,
                          variable_infos.role, variable_infos.component_count);
        block->field_add(field);
        // TODO: Add offset?????
      }
    }
    bool added = get_region()->add(block);
    if (!added) {
      delete block;
    }
  }
  // Check for results variables.

  //   // int num_attr = 0;
  //   // {
  //   //   Ioss::SerializeIO serializeIO__(this);
  //   //   int               ierr = ex_get_attr_param(get_file_pointer(), EX_NODE_BLOCK, 1,
  //   //   &num_attr); if (ierr < 0) {
  //   //     Ioex::exodus_error(get_file_pointer(), __LINE__, __func__, __FILE__);
  //   //   }
  //   // }
  //   //    add_attribute_fields(block, size, variables[block_name]);
  //   // add_attribute_fields(EX_NODE_BLOCK, block, num_attr, "");
  //   // add_results_fields(EX_NODE_BLOCK, block);

  //   bool added = get_region()->add(block);
  //   if (!added) {
  //     delete block;
  //   }
  //   // Load all fields listed in variables_map
  // } // namespace Ioad

  template <typename T>
  BlockInfoType DatabaseIO::get_variable_infos(adios2::IO &bpio, const std::string &var_name) const
  {
    BlockInfoType infos;

    auto   v    = bpio.InquireVariable<T>(var_name);
    size_t ndim = v.Shape().size();
    if (ndim != 3) {
      std::ostringstream errmsg;
      errmsg << "ERROR: BP variable dimension should be 3.\n";
      IOSS_ERROR(errmsg);
    }
    // For non-transient variables, not all blocks need to be loaded. Might improve speed.
    std::map<size_t, std::vector<typename adios2::Variable<T>::Info>> allblocks =
        bp_engine.AllStepsBlocksInfo(v);
    if (allblocks.empty()) {
      std::ostringstream errmsg;
      errmsg << "ERROR: Empty BP variable\n";
      IOSS_ERROR(errmsg);
    }
    size_t laststep = allblocks.rbegin()->first;
    // Only transient fields can have steps.
    adios2::Attribute<int> role_attr =
        bpio.InquireAttribute<int>(role_attribute, var_name, attribute_separator);
    infos.role = static_cast<Ioss::Field::RoleType>(role_attr.Data()[0]);
    if (infos.role != Ioss::Field::RoleType::TRANSIENT && laststep != 0) {
      std::ostringstream errmsg;
      errmsg << "ERROR: Last step should be 0 for non-transient fields. "
             << "Something is wrong in the Ioad::DatabaseIO::get_variable_size() function.\n";
      IOSS_ERROR(errmsg);
    }

    // Get VariableType
    adios2::Attribute<std::string> var_type_attr =
        bpio.InquireAttribute<std::string>(var_type_attribute, var_name, attribute_separator);
    infos.variable_type = var_type_attr.Data()[0];

    bool first = true;
    for (auto &blockpair : allblocks) {
      std::vector<typename adios2::Variable<T>::Info> &blocks = blockpair.second;
      // Sanity checks
      if (blocks[rank].Start[2] != 0) {
        std::ostringstream errmsg;
        errmsg << "ERROR: Number of components should always start at 0.\n";
        IOSS_ERROR(errmsg);
      }
      if (!blocks[rank].Count[0] || !blocks[rank].Count[1] || !blocks[rank].Count[2]) {
        std::ostringstream errmsg;
        errmsg << "ERROR: Block information does not contain Count.\n";
        IOSS_ERROR(errmsg);
      }
      // Only query the block size of the current process based on the process rank.
      // Skipping dimension=0 which is equal to the rank. Dimension=1 encodes beginning and count of
      // node count for this variable. Dimension=2 encodes the number of components for this
      // variables, and should always start at 0;
      size_t block_component_count       = blocks[rank].Count[2];
      size_t block_node_boundaries_start = blocks[rank].Start[1];
      size_t block_node_boundaries_size  = blocks[rank].Count[1];
      if (first) {
        infos.node_boundaries_start = block_node_boundaries_start;
        infos.node_boundaries_size  = block_node_boundaries_size;
        infos.component_count       = block_component_count;
      }
      else if (block_node_boundaries_start != infos.node_boundaries_start ||
               block_node_boundaries_size != infos.node_boundaries_size ||
               block_component_count != infos.component_count) {
        std::ostringstream errmsg;
        errmsg << "ERROR: Variable changes sizes over steps. Not supported by Ioad::DatabaseIO.\n";
        IOSS_ERROR(errmsg);
      }
      // Steps
      infos.steps.push_back(blockpair.first);
    }
    infos.basic_type = template_to_basic_type<T>();
    return infos;
  }

  //   void DatabaseIO::read_region(adios2::IO &bpio)
  //   {

  //     // Only get schema version attribute as it is the only one we expect.
  //     auto schema_version = bpio.InquireAttribute<unsigned int>(schema_version_string);
  //     if (!schema_version) {
  //       std::ostringstream errmsg;
  //       errmsg << "INTERNAL ERROR: schema_version_string not found. "
  //              << "Something is wrong in the Ioad::DatabaseIO::read_region() function. "
  //              << "Please check input file.\n";
  //       IOSS_ERROR(errmsg);
  //     }
  //     // Fow now, we do not do anything special based on the schema version. It is only used to
  //     check
  //     // that the input file is in the expected format.

  //     for (const auto &vpair : variables) {
  //       const std::string &name = vpair.first;
  //       std::cout << "name:" << name << std::endl;

  //       std::cout << "type:" << vpair.second.at("Type") << std::endl;
  //       std::cout << "shape:" << vpair.second.at("Shape") << std::endl;
  //       std::cout << "step counts:" << vpair.second.at("AvailableStepsCount") << std::endl;

  //       // Simply to start the "else if" section with "if".
  //       if (vpair.second.at("Type") == "not supported") {
  //       }
  // #define declare_template_instantiation(T)                             \
//   else if (vpair.second.at("Type") == adios2::helper::GetType<T>()) { \
//       read_variable_size<T>(bpio, vpair);                             \
//       }
  //       ADIOS2_FOREACH_TYPE_1ARG(declare_template_instantiation)
  // #undef declare_template_instantiation
  //     std::cout<<"vvvvvvvvvvvvvvvv"<<std::endl;
  //     // Once everything is loaded, get global variables such as spatialDimension.
  //     }
  //   }

  void DatabaseIO::read_meta_data__()
  {
    std::cout << "read" << std::endl;
    adios2::IO reader_io = ad->AtIO("reader");
    // Only get schema version attribute as it is the only one we expect.
    auto schema_version = reader_io.InquireAttribute<unsigned int>(schema_version_string);
    if (!schema_version) {
      std::ostringstream errmsg;
      errmsg << "ERROR: schema_version_string not found.\n";
      IOSS_ERROR(errmsg);
    }
    // Get all variables
    VariableMapType variables_map;

    const std::map<std::string, std::map<std::string, std::string>> variables =
        reader_io.AvailableVariables();
    std::string entity_type, entity_name, field_name;
    for (const auto &vpair : variables) {
      const std::string &name                        = vpair.first;
      std::tie(entity_type, entity_name, field_name) = decode_field_name(name);
      // We know this set of keys is unique as it is decoded from the variable name
      // and two varaibles cannot have the same name.
      variables_map[entity_type][entity_name][field_name] =
          std::make_pair(name, vpair.second.at("Type"));
    }

    // read_region(reader_io);
    std::cout << "======================================" << std::endl;
    // read_communication_metadata();

    // get_step_times__();
    // get_nodeblocks(reader_io, variables_map);
    get_nodeblocks(reader_io, variables_map);
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

  int64_t DatabaseIO::get_field_internal(const Ioss::Region *reg, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    throw "Not implemented yet";
    return 0;
  }
  int64_t DatabaseIO::get_field_internal(const Ioss::NodeBlock *nb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return get_field_internal(nb->type_string(), nb->name(), field, data, data_size);
  }
  int64_t DatabaseIO::get_field_internal(const Ioss::EdgeBlock *eb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    throw "Not implemented yet";
    return 0;
  }
  int64_t DatabaseIO::get_field_internal(const Ioss::FaceBlock *fb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    throw "Not implemented yet";
    return 0;
  }
  int64_t DatabaseIO::get_field_internal(const Ioss::ElementBlock *eb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    throw "Not implemented yet";
    return 0;
  }
  int64_t DatabaseIO::get_field_internal(const Ioss::StructuredBlock *sb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    throw "Not implemented yet";
    return 0;
  }
  int64_t DatabaseIO::get_field_internal(const Ioss::SideBlock *sb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    throw "Not implemented yet";
    return 0;
  }
  int64_t DatabaseIO::get_field_internal(const Ioss::NodeSet *ns, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    throw "Not implemented yet";
    return 0;
  }
  int64_t DatabaseIO::get_field_internal(const Ioss::EdgeSet *es, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    throw "Not implemented yet";
    return 0;
  }
  int64_t DatabaseIO::get_field_internal(const Ioss::FaceSet *fs, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    throw "Not implemented yet";
    return 0;
  }
  int64_t DatabaseIO::get_field_internal(const Ioss::ElementSet *es, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    throw "Not implemented yet";
    return 0;
  }
  int64_t DatabaseIO::get_field_internal(const Ioss::SideSet *ss, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    throw "Not implemented yet";
    return 0;
  }
  int64_t DatabaseIO::get_field_internal(const Ioss::CommSet *cs, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    throw "Not implemented yet";
    return 0;
  }

  int64_t DatabaseIO::get_field_internal(const std::string &entity_type,
                                         const std::string &entity_name, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    const std::string &field_name = field.get_name();

    adios2::IO  bpio         = ad->AtIO("writer"); // TODO: Update this!!!!!!!!!!!!!!!!
    std::string encoded_name = encode_field_name(entity_type, entity_name, field_name);

    switch (field.get_type()) {
    case Ioss::Field::BasicType::DOUBLE:
      return get_data<double>(bpio, field, data, encoded_name, data_size);
      break;
    case Ioss::Field::BasicType::INT32:
      return get_data<int32_t>(bpio, field, data, encoded_name, data_size);
      break;
    case Ioss::Field::BasicType::INT64:
      return get_data<int64_t>(bpio, field, data, encoded_name, data_size);
      break;
    case Ioss::Field::BasicType::COMPLEX:
      return get_data<Complex>(bpio, field, data, encoded_name, data_size);
      break;
    case Ioss::Field::BasicType::CHARACTER:
      return get_data<char>(bpio, field, data, encoded_name, data_size);
      break;
    default:
      std::ostringstream errmsg;
      errmsg << "INTERNAL ERROR: Invalid field type. "
             << "Something is wrong in the Ioad::DatabaseIO::get_field_internal() function. "
             << "Please report.\n";
      IOSS_ERROR(errmsg);
    }
    return 0;
  }

  template <typename T>
  int64_t DatabaseIO::get_data(adios2::IO &bpio, const Ioss::Field &field, void *data,
                               const std::string &encoded_name, size_t data_size) const
  {
    adios2::Variable<T> entities   = bpio.InquireVariable<T>(encoded_name);
    int                 num_to_get = field.verify(data_size);

    if (entities && data && data_size) {

      std::cout << "Get:" << encoded_name << std::endl;
      T *rdata = static_cast<T *>(data);
      bp_engine.Get<T>(entities, rdata,
                       adios2::Mode::Sync); // If not Sync, variables are not saved correctly.
      return num_to_get;
    }
    return 0;
  }

  // int64_t DatabaseIO::put_field_internal(const Ioss::StructuredBlock *sb, const Ioss::Field
  // &field,
  //                                        void *data, size_t data_size) const
  // {
  //   put_field_internal(sb->type_string(), field, data, data_size);
  // }

} // namespace Ioad

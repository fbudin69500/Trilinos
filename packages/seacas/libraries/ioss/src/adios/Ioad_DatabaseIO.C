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

#include <tokenize.h>

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
//#include "Ioss_StructuredBlock.h" // for StructuredBlock
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

#include <climits>

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

  // Constant variables
  const std::string                                  Schema_version_string = "IOSS_adios_version";
  const std::string                                  Sideblock_separator   = "::";
  const std::string                                  Name_separator        = "/";
  const std::string                                  Role_meta             = "role";
  const std::string                                  Var_type_meta         = "var_type";
  const std::string                                  Time_scale_factor     = "timeScaleFactor";
  const std::string                                  Time_meta             = "time";
  const std::string                                  globals_entity_type   = "globals";
  const std::string                                  globals_entity_name   = "";
  constexpr const char*                                  sideblock_names       = "sideblock_names_";
  constexpr size_t sideblock_names_size = strlen(sideblock_names);
  const std::map<std::string, std::set<std::string>> Use_transformed_storage_map = {
      {"ElementBlock", {"connectivity_edge", "connectivity_face"}},
      {"FaceBlock", {"connectivity_edge"}}};
  const std::map<std::string, std::set<std::string>> Ignore_fields = {
      {"NodeBlock",
       {"connectivity", "connectivity_raw", "node_connectivity_status", "implicit_ids"}},
      {"ElementBlock", {"implicit_ids"}},
      {"FaceBlock", {"connectivity_raw"}},
      {"EdgeBlock", {"connectivity_raw"}},
      {"CommSet", {"ids"}},
      {"SideSet", {"ids"}},
      {"SideBlock", {"side_ids", "ids", "connectivity", "connectivity_raw"}}};

} // namespace

namespace Ioad {

  DatabaseIO::DatabaseIO(Ioss::Region *region, const std::string &filename,
                         Ioss::DatabaseUsage db_usage, MPI_Comm communicator,
                         const Ioss::PropertyManager &properties_x)
      : Ioss::DatabaseIO(region, filename, db_usage, communicator, properties_x), rank(RankInit()),
        adios_wrapper(communicator, filename, is_input(), rank)
  {
    dbState = Ioss::STATE_UNKNOWN;
    // Always 64 bits
    dbIntSizeAPI = Ioss::USE_INT64_API;
  }

  // Used to force `rank` initialization before creating `adios_wrapper`.
  int DatabaseIO::RankInit()
  {
    Ioss::SerializeIO serializeIO__(this);
    number_proc = Ioss::SerializeIO::getSize();
    return Ioss::SerializeIO::getRank();
  }

  DatabaseIO::~DatabaseIO() {}

  bool DatabaseIO::begin__(Ioss::State state)
  {
    dbState = state;
    if (state == Ioss::STATE_MODEL) {
      adios_wrapper.BeginStep();
    }
    return true;
  }

  bool DatabaseIO::end__(Ioss::State state)
  {
    // Transitioning out of state 'state'
    assert(state == dbState);
    switch (state) {
    case Ioss::STATE_DEFINE_MODEL:
      if (!is_input()) {
        define_model();
        define_global_variables();
      }
      break;
    case Ioss::STATE_DEFINE_TRANSIENT:
      if (!is_input()) {
        Ioss::Field::RoleType role = Ioss::Field::RoleType::TRANSIENT;
        define_model(&role);
      }
      break;
    default: // ignore everything else...
      break;
    }

    {
      dbState = Ioss::STATE_UNKNOWN;
    }

    return true;
  }

  bool DatabaseIO::begin_state__(Ioss::Region * /* region */, int state, double time)
  {
    if (!is_input()) {
      // Begin  step for transient data
      adios_wrapper.BeginStep();
      // Add time to adios
      adios2::Variable<double> time_var = adios_wrapper.InquireVariable<double>(Time_meta);
      if (time_var) {
        adios_wrapper.PutMetaVariable<double>(Time_meta, time / timeScaleFactor);
      }
      else {
        std::ostringstream errmsg;
        errmsg << "ERROR: Time variable not defined.\n";
        IOSS_ERROR(errmsg);
      }
    }
    else {
      // TODO: Figure out if something needs to be done here.
      // Store reduction variables
      // read_reduction_fields();
    }
  }

  // common
  bool DatabaseIO::end_state__(Ioss::Region * /*region*/, int state, double time)
  {
    if (!is_input()) {
      // End step for transient data
      adios_wrapper.EndStep();
    }
    return true;
  }

  bool DatabaseIO::use_transformed_storage(const Ioss::Field &field, const std::string &entity_type,
                                           const std::string &field_name) const
  {
    if (field.get_role() == Ioss::Field::RoleType::TRANSIENT ||
        find_field_in_mapset(entity_type, field_name, Use_transformed_storage_map)) {
      return true;
    }
    return false;
  }

  template <typename T>
  void DatabaseIO::define_model_internal(const Ioss::Field &field, const std::string &encoded_name,
                                         const std::string &entity_type,
                                         const std::string &field_name)
  {
    size_t component_count;
    size_t local_size;

    if (use_transformed_storage(field, entity_type, field_name)) {
      component_count = field.transformed_storage()->component_count();
      local_size      = field.transformed_count();
    }
    else {
      component_count = field.raw_storage()->component_count();
      local_size      = field.raw_count();
    }
    adios_wrapper.DefineVariable<T>(encoded_name, {number_proc, INT_MAX, component_count},
                                    {rank, 0, 0}, {1, local_size, component_count});

    adios_wrapper.DefineMetaVariable<int>(Role_meta, encoded_name);
    adios_wrapper.DefineMetaVariable<std::string>(Var_type_meta, encoded_name);
  }

  std::string DatabaseIO::encode_field_name(const std::string &entity_type,
                                            const std::string &entity_name,
                                            const std::string &field_name) const
  {
    return entity_type + Name_separator + entity_name + Name_separator + field_name;
  }

  template <typename T>
  void DatabaseIO::define_entity_internal(const T &entity_blocks, Ioss::Field::RoleType *role)
  {
    for (auto entity_block : entity_blocks) {
      std::string entity_type = entity_blocks[0]->type_string();
      std::string entity_name = entity_blocks[0]->name();
      Ioss::NameList field_names;
      entity_block->field_describe(&field_names);
      for (auto field_name : field_names) {
        // Skip ignored fields
        if (find_field_in_mapset(entity_type, field_name, Ignore_fields)) {
          continue;
        }
        // Define entity block variables
        auto field = entity_block->get_fieldref(field_name);
        if (role && field.get_role() != *role) {
          continue;
        }
        std::string encoded_name = encode_field_name(entity_type, entity_name, field_name);
        switch (field.get_type()) {
        // REAL and DOUBLE have same value. Code is left here for documentation purposes
        // but commented out to compile.
        // case Ioss::Field::BasicType::REAL:
        case Ioss::Field::BasicType::DOUBLE:
          define_model_internal<double>(field, encoded_name, entity_type, field_name);
          break;
        case Ioss::Field::BasicType::INT32:
          define_model_internal<int32_t>(field, encoded_name, entity_type, field_name);
          break;
        case Ioss::Field::BasicType::INT64:
          define_model_internal<int64_t>(field, encoded_name, entity_type, field_name);
          break;
        case Ioss::Field::BasicType::COMPLEX:
          define_model_internal<Complex>(field, encoded_name, entity_type, field_name);
          break;
        case Ioss::Field::BasicType::CHARACTER:
          define_model_internal<char>(field, encoded_name, entity_type, field_name);
          break;
        case Ioss::Field::BasicType::STRING:
          adios_wrapper.DefineVariable<std::string>(
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


  std::string DatabaseIO::encode_sideblock_name(std::string type_string, std::string name) const
  {
    return encode_field_name(type_string, name, sideblock_names + std::to_string(rank));
  }

  bool DatabaseIO::is_sideblock_name(std::string name) const
  {
    size_t pos = name.find(sideblock_names);
    if(pos == 0) {
      unsigned long sb_rank = std::stoul(name.c_str()+sideblock_names_size);
      if(sb_rank == rank)
      {
        return true;
      }
    }
    return false;
  }

  // Similar to `write_meta_data()` function in other DatabaseIO. This function has been renamed in
  // this database to reflect more precisely what it accomplishes.
  void DatabaseIO::define_model(Ioss::Field::RoleType *role)
  {

    Ioss::Region *                  region      = get_region();
    const Ioss::NodeBlockContainer &node_blocks = region->get_node_blocks();

    // A single nodeblock named "nodeblock_1" will be created for the mesh. It contains information
    // for every node that exists in the model (Ioss-exodus-mapping.pdf).
    assert(node_blocks.size() == 1);
    // int spatialDimension = node_blocks[0]->get_property("component_degree").get_int();

    adios2::Attribute<unsigned int> schema_attr =
        adios_wrapper.InquireAttribute<unsigned int>(Schema_version_string);
    if (!schema_attr) {
      adios_wrapper.DefineAttribute<unsigned int>(Schema_version_string, 1);
    }
    define_entity_internal<Ioss::NodeBlockContainer>(node_blocks, role);
    // Edge Blocks --
    const Ioss::EdgeBlockContainer &edge_blocks = region->get_edge_blocks();
    define_entity_internal<Ioss::EdgeBlockContainer>(edge_blocks, role);
    // Face Blocks --
    const Ioss::FaceBlockContainer &face_blocks = region->get_face_blocks();
    define_entity_internal<Ioss::FaceBlockContainer>(face_blocks, role);
    // Element Blocks --
    const Ioss::ElementBlockContainer &element_blocks = region->get_element_blocks();
    define_entity_internal<Ioss::ElementBlockContainer>(element_blocks, role);
    // Nodesets ...
    const Ioss::NodeSetContainer &nodesets = region->get_nodesets();
    define_entity_internal<Ioss::NodeSetContainer>(nodesets, role);
    // Edgesets ...
    const Ioss::EdgeSetContainer &edgesets = region->get_edgesets();
    define_entity_internal<Ioss::EdgeSetContainer>(edgesets, role);
    // Facesets ...
    const Ioss::FaceSetContainer &facesets = region->get_facesets();
    define_entity_internal<Ioss::FaceSetContainer>(facesets, role);
    // Elementsets ...
    const Ioss::ElementSetContainer &elementsets = region->get_elementsets();
    define_entity_internal<Ioss::ElementSetContainer>(elementsets, role);
    // SideSets ...
    const Ioss::SideSetContainer &ssets = region->get_sidesets();
    define_entity_internal<Ioss::SideSetContainer>(ssets, role);
    // SideBlocks ...
    for (auto &sset : ssets) {
      // Until  local strings or GlobalArray of strings are supported, use multiple GlobalValue
      // variables, one per process (rank), to avoid having to create a char GlobalArray with extra
      // dimensions and fixed size components. In practice, the name of the side_blocks are unlikely
      // to change during computation, but this implementation allows us to not have to make any
      // assumption.
      std::string encoded_name                = encode_sideblock_name(sset->type_string(), sset->name());

      const Ioss::SideBlockContainer &sblocks = sset->get_side_blocks();
      adios2::Variable<std::string>   sblocks_var =
          adios_wrapper.InquireVariable<std::string>(encoded_name);
      if (!sblocks_var) {
        adios_wrapper.DefineVariable<std::string>(encoded_name);
        std::string stringified_sblock_names;
        for (auto sblock : sblocks) {
          stringified_sblock_names += Sideblock_separator + sblock->name();
        }
        adios_wrapper.Put<std::string>(
            encoded_name, stringified_sblock_names,
            adios2::Mode::Sync); // If not Sync, variables are not saved correctly.
      }
      define_entity_internal<Ioss::SideBlockContainer>(sblocks, role);
    }
  }

  void DatabaseIO::define_global_variables()
  {
    adios_wrapper.DefineAttribute<double>(Time_scale_factor, timeScaleFactor);
    adios_wrapper.DefineMetaVariable<double>(Time_meta);
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

  template <typename T>
  void DatabaseIO::put_data(const Ioss::Field &field, void *data,
                               const std::string &encoded_name) const
  {
    adios2::Variable<T> entities   = adios_wrapper.InquireVariable<T>(encoded_name);
    if (entities) {
      T *rdata = static_cast<T *>(data);
      adios_wrapper.Put<T>(entities, rdata,
                           adios2::Mode::Sync); // If not Sync, variables are not saved correctly.
    }
    else {
        std::ostringstream errmsg;
        errmsg << "ERROR: Could not find variable '" << encoded_name << "'\n";
        IOSS_ERROR(errmsg);
    }
  }

  int64_t DatabaseIO::put_field_internal(const std::string &entity_type,
                                         const std::string &entity_name, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    if (!data || !data_size) {
      return 0;
    }
    const std::string &field_name = field.get_name();
    if (find_field_in_mapset(entity_type, field_name, Ignore_fields)) {
      return 0;
    }

    int num_to_get = field.verify(data_size);
    if (num_to_get > 0) {

      std::string encoded_name      = encode_field_name(entity_type, entity_name, field_name);
      bool        transformed_field = use_transformed_storage(field, entity_type, field_name);
      int64_t     result            = 0;
      switch (field.get_type()) {
      case Ioss::Field::BasicType::DOUBLE:
        put_data<double>(field, data, encoded_name);
        break;
      case Ioss::Field::BasicType::INT32:
        put_data<int32_t>(field, data, encoded_name);
        break;
      case Ioss::Field::BasicType::INT64:
        put_data<int64_t>(field, data, encoded_name);
        break;
      case Ioss::Field::BasicType::COMPLEX:
        put_data<Complex>(field, data, encoded_name);
        break;
      case Ioss::Field::BasicType::CHARACTER:
        put_data<char>(field, data, encoded_name);
        break;
      default:
        std::ostringstream errmsg;
        errmsg << "INTERNAL ERROR: Invalid field type. "
               << "Something is wrong in the Ioad::DatabaseIO::put_field_internal() function. "
               << "Please report.\n";
        IOSS_ERROR(errmsg);
      }
      adios_wrapper.PutMetaVariable<int>(Role_meta, field.get_role(), encoded_name);
      std::string var_type =
          transformed_field ? field.transformed_storage()->name() : field.raw_storage()->name();
      adios_wrapper.PutMetaVariable<std::string>(Var_type_meta, var_type, encoded_name);
    }
    return num_to_get;
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
      std::string                   encoded_name = encode_field_name(ss->type_string(), ss->name(),
                                                   "sideblock_names" + std::to_string(rank));// TODO: remove rank since variable and not attribute.
      adios2::Variable<std::string> entities =
          adios_wrapper.InquireVariable<std::string>(encoded_name);
      std::vector<std::string> block_members;
      for (auto &sb : ss->get_side_blocks()) {
        block_members.push_back(sb->name());
      }
      if (entities && block_members.size()) {
        std::string stringified_block_members;
        for (std::string s : block_members) {
          stringified_block_members += "/" + s;
        }
        adios_wrapper.Put<std::string>(
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
    const std::string DatabaseIO::get_entity_type()
    {
      // Use "node" as default entity type to enable factory to create object.
      std::unique_ptr<T> entity(NewEntity<T>(this, "", "node", 0));
      return entity->type_string();
    }

    // NodeBlock has a different constructor...
    template <>
    const std::string DatabaseIO::get_entity_type<Ioss::NodeBlock>()
    {
      Ioss::NodeBlock nodeblock(this, "", 0, 1);
      return nodeblock.type_string();
    }

    // SideSet has a different constructor...
    template <>
    const std::string DatabaseIO::get_entity_type<Ioss::SideSet>()
    {
      Ioss::SideSet sideset(this, "");
      return sideset.type_string();
    }

    // SideBlock has a different constructor...
    template <>
    const std::string DatabaseIO::get_entity_type<Ioss::SideBlock>()
    {
      // default element is "hex8" because why not: We need an element type
      // to be able to construct this sideblock.
      Ioss::SideBlock sideblock(this, "", "node", "hex8", 0);
      return sideblock.type_string();
    }

  template <typename T>
  DatabaseIO::BlockInfoType DatabaseIO::get_variable_infos_from_map(
      const EntityMapType &entity_map, const std::string &entity_type,
      const std::string &block_name, const std::string &var_name) const
  {
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
    return get_variable_infos_from_map_no_check(entity_map, entity_type, block_name, var_name);
  }

  DatabaseIO::BlockInfoType DatabaseIO::get_variable_infos_from_map_no_check(
      const EntityMapType &entity_map, const std::string &entity_type,
      const std::string &block_name, const std::string &var_name) const
  {
    // No check to verify that the block_name, entity_type, variable_name, and type exist as they
    // have been extracted earlier from the file and everything should still be up to date.
    std::string variable = encode_field_name(entity_type, block_name, var_name);
    std::string type     = entity_map.at(block_name).at(var_name).second;
    // Simply to start the "else if" section with "if".
    if (type == "not supported") {
    }
#define declare_template_instantiation(T)                                                          \
  else if (type == adios2::helper::GetType<T>()) { return get_variable_infos<T>(variable); }
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

  template <typename T> T DatabaseIO::get_attribute(const std::string &attribute_name)
  {
    adios2::Attribute<T> attribute = adios_wrapper.InquireAttribute<T>(attribute_name);
    if (!attribute) {
      std::ostringstream errmsg;
      errmsg << "ERROR: " << attribute_name << " not found.\n";
      IOSS_ERROR(errmsg);
    }
    return attribute.Data()[0];
  }

  template<>
  void DatabaseIO::get_entities<Ioss::NodeBlock>(const VariableMapType &variables_map)
  {
    const std::string block_type = get_entity_type<Ioss::NodeBlock>();
    // For exodusII, there is only a single node block which contains
    // all of the nodes.
    // The default id assigned is '1' and the name is 'nodeblock_1'
    const EntityMapType &entity_map  = variables_map.at(block_type);
    std::string          block_name  = "nodeblock_1";

    // `mesh_model_coordinates` field is automatically created in NodeBlock constructor.
    std::string coord_var_name = "mesh_model_coordinates";
    // Get `node_count` and `spatialDimension`.
    BlockInfoType model_coordinates_infos =
        get_variable_infos_from_map<double>(entity_map, block_type, block_name, coord_var_name);
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
    block->property_add(Ioss::Property("id", 1));
    block->property_add(Ioss::Property("guid", util().generate_guid(1)));
    for (auto &variable : entity_map.at(block_name)) {
      // Since some fields are created automatically, we need to avoid recreating them when loading
      // the file.
      if (!block->field_exists(variable.first)) {
        BlockInfoType variable_infos = get_variable_infos_from_map_no_check(
            entity_map, block_type, block_name, variable.first);
        Ioss::Field field(variable.first, variable_infos.basic_type, variable_infos.variable_type,
                          variable_infos.role, variable_infos.node_boundaries_size);
        block->field_add(field);
        // TODO: Add offset?????
      }
    }
    bool added = get_region()->add(block);
    if (!added) {
      delete block;
    }
  }

  template <>
  void DatabaseIO::get_entities<Ioss::SideSet>(const VariableMapType &variables_map)
  {
    std::string entity_type = get_entity_type<Ioss::SideSet>();
    if (variables_map.find(entity_type) == variables_map.end()) {
      return;
    }
    std::string sideblock_type = get_entity_type<Ioss::SideBlock>();
    // Do SideBlock exist in the variables_map. This will be used when loading the SideBlock
    // for each SideSet, but we check this here outside of a loop as it only needs to be done once.
    bool no_sideblocks = (variables_map.find(sideblock_type) == variables_map.end());

    const EntityMapType &entity_map = variables_map.at(entity_type);
    for (auto &entity : entity_map) {
      std::string entity_name = entity.first;
      Ioss::SideSet *ss = new Ioss::SideSet(this, entity_name);
      bool           added = get_region()->add(ss);
      if (!added) {
        delete ss;
        return;
      }
      for (auto &variable_pair : entity.second) {
        // Since some fields are created automatically, we need to avoid recreating them when
        // loading the file.
        // First, check that field is actually a field and not a list of sideblocks
        if (!is_sideblock_name(variable_pair.first)) {
          if (!ss->field_exists(variable_pair.first)) {
            BlockInfoType infos = get_variable_infos_from_map_no_check(
                entity_map, entity_type, entity_name, variable_pair.first);
            // TODO: Should we have a sanity check to verify that the field size is the same as
            // the entity size?
            Ioss::Field field(variable_pair.first, infos.basic_type, infos.variable_type,
                              infos.role, infos.node_boundaries_size);
            ss->field_add(field);
          }
        }
        else {
          // The field in the bp file is actually a list of sideblocks
          // If no sideblock, don't worry about this section and move on to the next
          // sideset.
          if(no_sideblocks) {
            continue;
          }
          // TODO: Use metavariables or arrays of char
          std::string encoded_name =
              encode_field_name(entity_type, entity_name, variable_pair.first);
          adios2::Variable<std::string> sideblock_var =
              adios_wrapper.InquireVariable<std::string>(encoded_name);
          if (sideblock_var) {
            std::string stringified_names;
            adios_wrapper.Get<std::string>(sideblock_var, stringified_names);
            for (std::string name : Ioss::tokenize(stringified_names, Sideblock_separator)) {
              // TODO: Get element topology!!!
              // Get information about first element in sideblock map.
              // Ioss::Sideblock *  sideblock = new Ioss::Sideblock(this, name,
              // first_field_infos.variable_type,
              //  first_field_infos.node_boundaries_size);
              if (variables_map.at(sideblock_type).find(name) != variables_map.at(sideblock_type).end()) {
                for (auto &sideblock_pair : variables_map.at(sideblock_type).at(name)) {
                  BlockInfoType infos = get_variable_infos_from_map_no_check(
                      entity_map, sideblock_type, name, sideblock_pair.first);
                }
              }
            }
          }
        }
      }
    }
  }

  // // Takes an extra unused parameter "entity_type" to match the API of the function
  // // "NewEntity" used for objects that are not EntitySets which require that parameter.
  // // Having matching APIs allows to call this function from generic templated functions
  // // that do not have to be specialized to call this function with a different number of
  // // parameters.
  // template <typename T>
  // typename std::enable_if<std::is_base_of<Ioss::EntitySet, T>::value, T>::type *
  // DatabaseIO::NewEntity(DatabaseIO *io_database, const std::string &my_name,
  //                       const std::string &/*entity_type*/, size_t entity_count)
  // {
  //   return new T(io_database, my_name, entity_count);
  // }

  // // Does not work for NodeBlock. Directly use NodeBlock constructor for NodeBlock
  // // objects.
  // template <typename T>
  // typename std::enable_if<!std::is_base_of<Ioss::EntitySet, T>::value, T>::type *
  // DatabaseIO::NewEntity(DatabaseIO *io_database, const std::string &my_name,
  //                       const std::string &entity_type, size_t entity_count)
  // {
  //   return new T(io_database, my_name, entity_type, entity_count);
  // }

  template <typename T>
  auto
  DatabaseIO::NewEntity(DatabaseIO *io_database, const std::string &my_name,
                        const std::string &/*entity_type*/, size_t entity_count)
  -> decltype(T(io_database, my_name, entity_count)) *
  {
    return new T(io_database, my_name, entity_count);
  }

  template <typename T>
  auto DatabaseIO::NewEntity(DatabaseIO *io_database, const std::string &my_name,
                        const std::string &entity_type, size_t entity_count)
  -> decltype(T(io_database, my_name, entity_type, entity_count)) *
  {
    return new T(io_database, my_name, entity_type, entity_count);
  }

  template <typename T>
  void DatabaseIO::get_entities(const VariableMapType &variables_map)
  {
    std::string entity_type = get_entity_type<T>();
    if (variables_map.find(entity_type) == variables_map.end()) {
      return;
    }

    const EntityMapType &entity_map = variables_map.at(entity_type);
    for (auto &variable_pair : entity_map) {
      std::string entity_name = variable_pair.first;
      // Get size and type info for the entity using the first element in the map.
      BlockInfoType first_field_infos = get_variable_infos_from_map_no_check(
          entity_map, entity_type, entity_name, variable_pair.second.begin()->first);
      T *  entity = NewEntity<T>(this, entity_name, first_field_infos.variable_type,
                               first_field_infos.node_boundaries_size);
      bool added  = get_region()->add(entity);
      if (!added) {
        delete entity;
      }
      for (auto &variable_pair : variable_pair.second) {
        // Since some fields are created automatically, we need to avoid recreating them when
        // loading the file.
        // Note: We get the information about the first field twice: once before this loop, and
        // once inside the loop. The code could be modified to perform this action only once but
        // most likely the code will be more complex and will not save a lot of computation time.
        if (!entity->field_exists(variable_pair.first)) {
          BlockInfoType infos = get_variable_infos_from_map_no_check(
              entity_map, entity_type, entity_name, variable_pair.first);
          // TODO: Should we have a sanity check to verify that the field size is the same as
          // the entity size?
          Ioss::Field field(variable_pair.first, infos.basic_type, infos.variable_type, infos.role,
                            infos.node_boundaries_size);
          entity->field_add(field);
        }
      }
    }
  }


  template <typename T>
  DatabaseIO::BlockInfoType DatabaseIO::get_variable_infos(const std::string &var_name) const
  {
    BlockInfoType infos;

    auto   v    = adios_wrapper.InquireVariable<T>(var_name);
    size_t ndim = v.Shape().size();
    if (ndim != 3) {
      std::ostringstream errmsg;
      errmsg << "ERROR: BP variable dimension should be 3.\n";
      IOSS_ERROR(errmsg);
    }
    // For non-transient variables, not all blocks need to be loaded. Might improve speed.
    std::map<size_t, std::vector<typename adios2::Variable<T>::Info>> allblocks =
        adios_wrapper.AllStepsBlocksInfo(v);
    if (allblocks.empty()) {
      std::ostringstream errmsg;
      errmsg << "ERROR: Empty BP variable\n";
      IOSS_ERROR(errmsg);
    }
    size_t laststep = allblocks.rbegin()->first;
    // Only transient fields can have steps.
    infos.role =
        static_cast<Ioss::Field::RoleType>(adios_wrapper.GetMetaVariable<int>(Role_meta, var_name));
    if (infos.role != Ioss::Field::RoleType::TRANSIENT && laststep != 0) {
      std::ostringstream errmsg;
      errmsg << "ERROR: Last step should be 0 for non-transient fields. "
             << "Something is wrong in the Ioad::DatabaseIO::get_variable_size() function.\n";
      IOSS_ERROR(errmsg);
    }

    // Get VariableType
    infos.variable_type = adios_wrapper.GetMetaVariable<std::string>(Var_type_meta, var_name);

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
      // Skipping dimension=0 which is equal to the rank. Dimension=1 encodes beginning and count
      // of node count for this variable. Dimension=2 encodes the number of components for this
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

    void DatabaseIO::get_globals(const VariableMapType &variables_map)
    {
      const EntityMapType &entity_map = variables_map.at(globals_entity_type);

      // This should never happen as if there is a `globals` field added to the map, that means that
      // there are global fields inside it.
      assert(!entity_map.empty());

      timeScaleFactor = get_attribute<double>(Time_scale_factor);
      Ioss::SerializeIO serializeIO__(this);
      Ioss::Region *    this_region = get_region();
      auto              globals_map = entity_map.at(globals_entity_name);
      if (globals_map.find(Time_meta) != globals_map.end()) {
        // Load time steps
        // 1) Check that the time type is `double` as expected.
        adios2::Variable<double> time_var = adios_wrapper.InquireVariable<double>(Time_meta);
        std::vector<double>      tsteps(1);
        if (time_var) {
          for (size_t step = 0; step < time_var.Steps(); step++) {
            // if (tsteps[i] <= last_time) { TODO: Check last time step before writing everything
            time_var.SetStepSelection(std::make_pair(step, 1)); // Check with streaming.
            adios_wrapper.Get(time_var, tsteps.data());
            this_region->add_state__(tsteps[0] * timeScaleFactor);
          }
        }
        else {
          std::ostringstream errmsg;
          errmsg << "ERROR: Timestep global detected in file but cannot read it.\n";
          IOSS_ERROR(errmsg);
        }
      }
    }


    void DatabaseIO::read_meta_data__()
    {
      // Only get schema version attribute as it is the only one we expect.
      get_attribute<unsigned int>(Schema_version_string);

      // Get all variables
      VariableMapType variables_map;

      const std::map<std::string, std::map<std::string, std::string>> variables =
          adios_wrapper.AvailableVariables();
      for (const auto &vpair : variables) {
        const std::string &      name   = vpair.first;
        std::vector<std::string> tokens = Ioss::tokenize(name, Name_separator);
        if (tokens.size() == 1) {
          // Special case for global variables.
          tokens.insert(tokens.begin(), globals_entity_type);
          tokens.insert(tokens.begin() + 1, globals_entity_name);
        }
        else if (tokens.size() != 3) {
          // There is a problem as there should be 1 or 3 variables found in the encoded name.
          std::ostringstream errmsg;
          errmsg << "ERROR: Invalid encoded entity name. Does not contain 2 separators.\n";
          IOSS_ERROR(errmsg);
        }
        // Check if variable contains meta-data (field_name(=tokens[2]) contains meta_separator)
        tokens.push_back("");
        std::tie(tokens[2], tokens[3]) = adios_wrapper.DecodeMetaName(tokens[2]);
        // We know this set of keys is unique as it is decoded from the variable name
        // and two varaibles cannot have the same name. Do not save meta-variables.
        if (tokens[3].empty()) {
          variables_map[tokens[0]][tokens[1]][tokens[2]] =
              std::make_pair(name, vpair.second.at("Type"));
        }
      }

      // read_region(reader_io);
      // read_communication_metadata();

      get_globals(variables_map);
      get_entities<Ioss::NodeBlock>(variables_map);
      get_entities<Ioss::EdgeBlock>(variables_map);
      get_entities<Ioss::FaceBlock>(variables_map);
      get_entities<Ioss::ElementBlock>(variables_map);

      check_side_topology();

      get_entities<Ioss::SideSet>(variables_map);
      get_entities<Ioss::NodeSet>(variables_map);
      get_entities<Ioss::EdgeSet>(variables_map);
      get_entities<Ioss::FaceSet>(variables_map);
      get_entities<Ioss::ElementSet>(variables_map);
      get_entities<Ioss::CommSet>(variables_map);

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
      return get_field_internal(eb->type_string(), eb->name(), field, data, data_size);
    }
    int64_t DatabaseIO::get_field_internal(const Ioss::FaceBlock *fb, const Ioss::Field &field,
                                           void *data, size_t data_size) const
    {
      return get_field_internal(fb->type_string(), fb->name(), field, data, data_size);
    }
    int64_t DatabaseIO::get_field_internal(const Ioss::ElementBlock *eb, const Ioss::Field &field,
                                           void *data, size_t data_size) const
    {
      return get_field_internal(eb->type_string(), eb->name(), field, data, data_size);
    }
    int64_t DatabaseIO::get_field_internal(const Ioss::SideBlock *sb, const Ioss::Field &field,
                                           void *data, size_t data_size) const
    {
      return get_field_internal(sb->type_string(), sb->name(), field, data, data_size);
    }
    int64_t DatabaseIO::get_field_internal(const Ioss::NodeSet *ns, const Ioss::Field &field,
                                           void *data, size_t data_size) const
    {
      return get_field_internal(ns->type_string(), ns->name(), field, data, data_size);
    }
    int64_t DatabaseIO::get_field_internal(const Ioss::EdgeSet *es, const Ioss::Field &field,
                                           void *data, size_t data_size) const
    {
      return get_field_internal(es->type_string(), es->name(), field, data, data_size);
    }
    int64_t DatabaseIO::get_field_internal(const Ioss::FaceSet *fs, const Ioss::Field &field,
                                           void *data, size_t data_size) const
    {
      return get_field_internal(fs->type_string(), fs->name(), field, data, data_size);
    }
    int64_t DatabaseIO::get_field_internal(const Ioss::ElementSet *es, const Ioss::Field &field,
                                           void *data, size_t data_size) const
    {
      return get_field_internal(es->type_string(), es->name(), field, data, data_size);
    }
    int64_t DatabaseIO::get_field_internal(const Ioss::SideSet *ss, const Ioss::Field &field,
                                           void *data, size_t data_size) const
    {
      return get_field_internal(ss->type_string(), ss->name(), field, data, data_size);
    }
    int64_t DatabaseIO::get_field_internal(const Ioss::CommSet *cs, const Ioss::Field &field,
                                           void *data, size_t data_size) const
    {
      return get_field_internal(cs->type_string(), cs->name(), field, data, data_size);
    }

    int64_t DatabaseIO::get_field_internal(const std::string &entity_type,
                                           const std::string &entity_name, const Ioss::Field &field,
                                           void *data, size_t data_size) const
    {
      const std::string &field_name = field.get_name();

      std::string encoded_name = encode_field_name(entity_type, entity_name, field_name);

      switch (field.get_type()) {
      case Ioss::Field::BasicType::DOUBLE:
        return get_data<double>(field, data, encoded_name, data_size);
        break;
      case Ioss::Field::BasicType::INT32:
        return get_data<int32_t>(field, data, encoded_name, data_size);
        break;
      case Ioss::Field::BasicType::INT64:
        return get_data<int64_t>(field, data, encoded_name, data_size);
        break;
      case Ioss::Field::BasicType::COMPLEX:
        return get_data<Complex>(field, data, encoded_name, data_size);
        break;
      case Ioss::Field::BasicType::CHARACTER:
        return get_data<char>(field, data, encoded_name, data_size);
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
    int64_t DatabaseIO::get_data(const Ioss::Field &field, void *data,
                                 const std::string &encoded_name, size_t data_size) const
    {
      adios2::Variable<T> entities   = adios_wrapper.InquireVariable<T>(encoded_name);
      int                 num_to_get = field.verify(data_size);

      if (entities && data && data_size) {
        T *rdata = static_cast<T *>(data);
        // Set selection per rank. Support written by N processes, and loaded by M processes.
        adios_wrapper.Get<T>(entities, rdata,
                             adios2::Mode::Sync); // If not Sync, variables are not saved correctly.
        return num_to_get;
      }
      return 0;
    }

} // namespace Ioad

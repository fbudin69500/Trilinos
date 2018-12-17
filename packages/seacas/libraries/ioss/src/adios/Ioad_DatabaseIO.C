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

#include "Ioss_CommSet.h"        // for CommSet
#include "Ioss_DBUsage.h"        // for DatabaseUsage, etc
#include "Ioss_DatabaseIO.h"     // for DatabaseIO
#include "Ioss_EdgeBlock.h"      // for EdgeBlock
#include "Ioss_EdgeSet.h"        // for EdgeSet
#include "Ioss_ElementBlock.h"   // for ElementBlock
#include "Ioss_ElementSet.h"     // for ElementSet
#include "Ioss_EntityType.h"     // for EntityType::ELEMENTBLOCK
#include "Ioss_FaceBlock.h"      // for FaceBlock
#include "Ioss_FaceSet.h"        // for FaceSet
#include "Ioss_FileInfo.h"       // for FileInfo
#include "Ioss_GroupingEntity.h" // for GroupingEntity
#include "Ioss_Map.h"            // for Map, MapContainer
#include "Ioss_NodeBlock.h"      // for NodeBlock
//#include "Ioss_StructuredBlock.h" // for StructuredBlock
#include "Ioss_NodeSet.h"         // for NodeSet
#include "Ioss_Property.h"        // for Property
#include "Ioss_Region.h"          // for Region, SideSetContainer, etc
#include "Ioss_SideBlock.h"       // for SideBlock
#include "Ioss_SideSet.h"         // for SideBlockContainer, SideSet
//#include "Ioss_VariableType.h"    // for VariableType
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

namespace Ioad {

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
    const std::string     Schema_version_string = "IOSS_adios_version";
    const std::string     Sideblock_separator   = "::";
    const std::string     Name_separator        = "/";
    const std::string     Role_meta             = "role";
    const std::string     Var_type_meta         = "var_type";
    const std::string     Topology_meta         = "topology";
    const std::string     property_meta         = "property_";
    const std::string     Parent_topology_meta  = "parent_topology";
    const std::string     Time_scale_factor     = "time_scale_factor";
    const std::string     Time_meta             = "time";
    const std::string     Processor_id_meta     = "processor_id";
    const std::string     Processor_number_meta = "processor_number";
    const std::string     globals_entity_type   = "globals";
    const std::string     globals_entity_name   = "";
    const std::string     region_name           = "no_name";
    const std::string     original_name         = "original_name";
    constexpr const char *sideblock_names       = "sideblock_names";
    constexpr size_t      sideblock_names_size  = strlen(sideblock_names);
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
    const std::vector<std::string> Ignore_properties = {{"name", "_base_stk_part_name", "db_name"}};
  } // namespace

  DatabaseIO::DatabaseIO(Ioss::Region *region, const std::string &filename,
                         Ioss::DatabaseUsage db_usage, MPI_Comm communicator,
                         const Ioss::PropertyManager &props)
      : Ioss::DatabaseIO(region, filename, db_usage, communicator, props), rank(RankInit()),
        adios_wrapper(communicator, filename, is_input(), rank, props)
  {
    dbState = Ioss::STATE_UNKNOWN;
    // Always 64 bits
    dbIntSizeAPI = Ioss::USE_INT64_API;
    set_logging(false);
    is_streaming = adios_wrapper.IsStreaming();
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
    std::cout<<"begin__"<<state<<" " << Ioss::STATE_DEFINE_MODEL <<std::endl;
    dbState = state;
    //if (state == Ioss::STATE_MODEL) {
    if (state == Ioss::STATE_DEFINE_MODEL) {
      // Should `BeginStep()` only be if (!is_input() || is_streaming) ???

      adios_wrapper.BeginStep();
    }
    return true;
  }

  std::string DatabaseIO::get_property_variable_name(const std::string &property_name)
  {
    return property_meta + property_name;
  }

  void DatabaseIO::write_properties(const Ioss::GroupingEntity * const entity, const std::string & encoded_name)
  {
      // Write field properties
      std::vector<std::string> property_list = properties_to_save(entity);
      for (auto property_name : property_list) {
        Ioss::Property property = entity->get_property(property_name);

        std::string variable_name = get_property_variable_name(property_name);
        switch (property.get_type()) {
        case Ioss::Property::BasicType::REAL:
          adios_wrapper.PutMetaVariable<double>(variable_name, property.get_real(), encoded_name);
          break;
        case Ioss::Property::BasicType::INTEGER:
          adios_wrapper.PutMetaVariable<int64_t>(variable_name, property.get_int(), encoded_name);
          break;
        case Ioss::Property::BasicType::STRING:
          adios_wrapper.PutMetaVariable<std::string>(variable_name, property.get_string(),
                                                     encoded_name);
          break;
        case Ioss::Property::BasicType::POINTER:
        case Ioss::Property::BasicType::INVALID:
        default:
          // Do not save properties with an invalid type.
          break;
        }
      }
  }

// TODO: refactor and consolidate `write_meta_data_container` functions to avoid code duplication.
  template <typename T> void DatabaseIO::write_meta_data_container(const T &entity_blocks)
  {
    for (auto entity_block : entity_blocks) {
      std::string entity_type  = entity_block->type_string();
      std::string entity_name  = entity_block->name();
      std::string encoded_name = encode_field_name({entity_type, entity_name});
      write_properties(entity_block, encoded_name);
      adios_wrapper.PutMetaVariable<std::string>(Topology_meta, entity_block->topology()->name(),
                                               encoded_name);
    }
  }

  template <> void DatabaseIO::write_meta_data_container<Ioss::CommSetContainer>(const Ioss::CommSetContainer &entity_blocks)
  {
    for (auto entity_block : entity_blocks) {
      std::string entity_type  = entity_block->type_string();
      std::string entity_name  = entity_block->name();
      std::string encoded_name = encode_field_name({entity_type, entity_name});
      write_properties(entity_block, encoded_name);
    }
  }

  template <>
  void DatabaseIO::write_meta_data_container<Ioss::SideBlockContainer>(
      const Ioss::SideBlockContainer &entity_blocks)
  {
    for (auto entity_block : entity_blocks) {
      std::string entity_type  = entity_block->type_string();
      std::string entity_name  = entity_block->name();
      std::string encoded_name = encode_field_name({entity_type, entity_name});
      adios_wrapper.PutMetaVariable<std::string>(Topology_meta, entity_block->topology()->name(),
                                                 encoded_name);
      adios_wrapper.PutMetaVariable<std::string>(
          Parent_topology_meta, entity_block->parent_element_topology()->name(), encoded_name);
      write_properties(entity_block, encoded_name);
    }
  }

  std::string DatabaseIO::stringify_side_block_names(const Ioss::SideBlockContainer &sblocks) const
  {
    std::string stringified_sblock_names;
    for (auto sblock : sblocks) {
      stringified_sblock_names += Sideblock_separator + sblock->name();
    }
    return stringified_sblock_names;
  }

  template <>
  void DatabaseIO::write_meta_data_container<Ioss::SideSetContainer>(const Ioss::SideSetContainer &ssets)
  {
    for (auto &sset : ssets) {
      const std::string entity_type = sset->type_string();
      const std::string entity_name = sset->name();
      write_properties(sset, encode_field_name({entity_type, entity_name}));

      // side blocks
      const Ioss::SideBlockContainer &sblocks = sset->get_side_blocks();
      std::string encoded_name = encode_sideblock_name(entity_type, entity_name);

      std::string stringified_sblock_names;
      adios2::Variable<int8_t> sblocks_var =
          adios_wrapper.InquireVariable<int8_t>(encoded_name);
      // adios2::Variable<char> sblocks_var =
      //     adios_wrapper.InquireVariable<char>(encoded_name);
      if (sblocks_var) {
        std::string stringified_sblock_names = stringify_side_block_names(sblocks);
        adios_wrapper.Put<int8_t>(
            sblocks_var, reinterpret_cast<const int8_t*>(stringified_sblock_names.c_str()),
            adios2::Mode::Sync); // If not Sync, variables are not saved correctly.
          // adios_wrapper.Put<char>(
          //   sblocks_var, stringified_sblock_names.c_str(),
          //   adios2::Mode::Sync); // If not Sync, variables are not saved correctly.
      }
      else {
        std::ostringstream errmsg;
        errmsg << "ERROR: Could not find variable '" << encoded_name << "'\n";
        IOSS_ERROR(errmsg);
      }
      write_meta_data_container(sblocks);
    }
  }

  // Write data defined per block, not per field.
  void DatabaseIO::write_meta_data()
  {
    Ioss::Region *                  region      = get_region();
    // Region
    write_properties(region, encode_field_name({region->type_string(), region_name}));
    // Node blocks --
    const Ioss::NodeBlockContainer &node_blocks = region->get_node_blocks();
    write_meta_data_container<Ioss::NodeBlockContainer>(node_blocks);
    // Edge Blocks --
    const Ioss::EdgeBlockContainer &edge_blocks = region->get_edge_blocks();
    write_meta_data_container<Ioss::EdgeBlockContainer>(edge_blocks);
    // Face Blocks --
    const Ioss::FaceBlockContainer &face_blocks = region->get_face_blocks();
    write_meta_data_container<Ioss::FaceBlockContainer>(face_blocks);
    // Element Blocks --
    const Ioss::ElementBlockContainer &element_blocks = region->get_element_blocks();
    write_meta_data_container<Ioss::ElementBlockContainer>(element_blocks);
    // Side Blocks ...
    const Ioss::SideSetContainer &ssets = region->get_sidesets();
    write_meta_data_container<Ioss::SideSetContainer>(ssets);
    // Comm Sets --
    const Ioss::CommSetContainer &comm_sets = region->get_commsets();
    write_meta_data_container<Ioss::CommSetContainer>(comm_sets);
    // Global meta data
    put_data<unsigned long>(static_cast<void*>(&rank), Processor_id_meta);
    put_data<unsigned long>(static_cast<void*>(&number_proc), Processor_number_meta);
  }

  bool DatabaseIO::end__(Ioss::State state)
  {
    // Transitioning out of state 'state'
    std::cout<<"State: "<<state<<std::endl;
    assert(state == dbState);
    switch (state) {
    case Ioss::STATE_DEFINE_MODEL:
      if (!is_input()) {
        define_model();
        define_global_variables();
        write_meta_data();
      }
      break;
    case Ioss::STATE_DEFINE_TRANSIENT:
      if (!is_input()) {
        if(is_streaming) {
          // If streaming, we need to write the meta data available at
          // every step.
          write_meta_data();
        }
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
    std::cout<<"begin_state__"<<std::endl;
    if (!is_input()) {
      // `BeginStep()` should not be used at the same time as random access. Since at read time,
      // we currrently read variables with random access, `BeginStep()` should not be used
      // at read time.
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
      if(is_streaming) {
        std::cout<<"ISSTREAMING"<<std::endl;
          // Begin step for transient data if streaming. Otherwise, data will be accessed with
          // `SetStepSelection()`.
          adios_wrapper.BeginStep();
      }
      // TODO: Figure out if something needs to be done here.
      // Store reduction variables
      // read_reduction_fields();
    }
    return true;
  }

  // common
  bool DatabaseIO::end_state__(Ioss::Region * /*region*/, int state, double time)
  {
    std::cout<<"end_state__"<<std::endl;

    //if (!is_input()) {
      // End step for transient data
      adios_wrapper.EndStep();
    //}
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

  void DatabaseIO::define_field_meta_variables(const std::string &encoded_name)
  {
    adios_wrapper.DefineMetaVariable<int>(Role_meta, encoded_name);
    adios_wrapper.DefineMetaVariable<std::string>(Var_type_meta, encoded_name);
  }

  template <typename T, typename = DatabaseIO::IsIossEntityBlock<T>>
  void DatabaseIO::define_entity_meta_variables(const std::string &encoded_name)
  {
    adios_wrapper.DefineMetaVariable<std::string>(Topology_meta, encoded_name);
  }

  template <typename T, typename = DatabaseIO::IsNotIossEntityBlock<T>, typename = void>
  void DatabaseIO::define_entity_meta_variables(const std::string &encoded_name)
  {
    // no op
  }

  template <>
  void DatabaseIO::define_entity_meta_variables<Ioss::SideBlock>(const std::string &encoded_name)
  {
    adios_wrapper.DefineMetaVariable<std::string>(Topology_meta, encoded_name);
    adios_wrapper.DefineMetaVariable<std::string>(Parent_topology_meta, encoded_name);
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
  }

  std::string DatabaseIO::encode_field_name(std::vector<std::string> names) const
  {
    std::string encoded_name;
    size_t      count = 0;
    for (std::vector<std::string>::iterator it = names.begin(); it != names.end() - 1;
         it++, count++) {
      encoded_name += *it + Name_separator;
    }
    // sentinel value to avoid having to test if the string is empty or not.
    names.push_back("");
    return encoded_name + names[count];
  }

  std::vector<std::string>
  DatabaseIO::properties_to_save(const Ioss::GroupingEntity *const entity_block)
  {
    std::vector<std::string> property_list;
    entity_block->property_describe(&property_list);

    for (auto ignore_property : Ignore_properties) {
      property_list.erase(std::remove(property_list.begin(), property_list.end(), ignore_property),
                          property_list.end());
    }
    property_list.erase(std::remove_if(property_list.begin(), property_list.end(),
                                       [&](std::string property_name) -> bool {
                                         Ioss::Property property =
                                             entity_block->get_property(property_name);
                                         if (property.is_invalid() || property.is_implicit()) {
                                           return true;
                                         }
                                         else {
                                           return false;
                                         }
                                       }),
                        property_list.end());
    return property_list;
  }

  void DatabaseIO::define_properties(const Ioss::GroupingEntity *const entity_block,
                                     const std::string &               encoded_entity_name)
  {
    std::vector<std::string> property_list = properties_to_save(entity_block);
    for (auto property_name : property_list) {
      Ioss::Property property      = entity_block->get_property(property_name);
      std::string    variable_name = get_property_variable_name(property_name);
      switch (property.get_type()) {
      case Ioss::Property::BasicType::REAL:
        adios_wrapper.DefineMetaVariable<double>(variable_name, encoded_entity_name);
        break;
      case Ioss::Property::BasicType::INTEGER:
        adios_wrapper.DefineMetaVariable<int64_t>(variable_name, encoded_entity_name);
        break;
      case Ioss::Property::BasicType::STRING:
        adios_wrapper.DefineMetaVariable<std::string>(variable_name, encoded_entity_name);
        break;
      case Ioss::Property::BasicType::POINTER:
      case Ioss::Property::BasicType::INVALID:
      default:
        // Do not save properties with an invalid type.
        break;
      }
    }
  }

  template <typename T>
  void DatabaseIO::define_entity_internal(const T &entity_blocks, Ioss::Field::RoleType *role)
  {
    using cv_removed_value_type = typename std::remove_pointer<typename T::value_type>::type;
    for (auto entity_block : entity_blocks) {
      std::string entity_type = entity_block->type_string();
      std::string entity_name = entity_block->name();
      if (!role) {
        // Meta variables and properties are only defined if state is STATE_DEFINE_MODEL:
        std::string encoded_entity_name = encode_field_name({entity_type, entity_name});
        define_entity_meta_variables<cv_removed_value_type>(encoded_entity_name);
        define_properties(entity_block, encoded_entity_name);
      }
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
        std::string encoded_name = encode_field_name({entity_type, entity_name, field_name});
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
        define_field_meta_variables(encoded_name);
      }
    }
  }

  std::string DatabaseIO::encode_sideblock_name(std::string type_string, std::string name) const
  {
    return encode_field_name({type_string, name, sideblock_names});
  }

  bool DatabaseIO::is_sideblock_name(std::string name) const
  {
    size_t pos = name.find(sideblock_names);
    if (pos == 0) {
      return true;
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

    if (!role) {
      // Schema
      adios2::Attribute<unsigned int> schema_attr =
          adios_wrapper.InquireAttribute<unsigned int>(Schema_version_string);
      if (!schema_attr) {
        adios_wrapper.DefineAttribute<unsigned int>(Schema_version_string, 1);
      }
      // Region
      define_properties(region, encode_field_name({region->type_string(), region_name}));
    }
    // Node blocks --
    define_entity_internal(node_blocks, role);
    // Edge Blocks --
    const Ioss::EdgeBlockContainer &edge_blocks = region->get_edge_blocks();
    define_entity_internal(edge_blocks, role);
    // Face Blocks --
    const Ioss::FaceBlockContainer &face_blocks = region->get_face_blocks();
    define_entity_internal(face_blocks, role);
    // Element Blocks --
    const Ioss::ElementBlockContainer &element_blocks = region->get_element_blocks();
    define_entity_internal(element_blocks, role);
    // Nodesets ...
    const Ioss::NodeSetContainer &nodesets = region->get_nodesets();
    define_entity_internal(nodesets, role);
    // Edgesets ...
    const Ioss::EdgeSetContainer &edgesets = region->get_edgesets();
    define_entity_internal(edgesets, role);
    // Facesets ...
    const Ioss::FaceSetContainer &facesets = region->get_facesets();
    define_entity_internal(facesets, role);
    // Elementsets ...
    const Ioss::ElementSetContainer &elementsets = region->get_elementsets();
    define_entity_internal(elementsets, role);
    // CommSets ...
    const Ioss::CommSetContainer &csets = region->get_commsets();
    define_entity_internal(csets, role);
    // SideSets ...
    const Ioss::SideSetContainer &ssets = region->get_sidesets();
    define_entity_internal(ssets, role);
    // SideBlocks ...
    for (auto &sset : ssets) {
      std::string encoded_name = encode_sideblock_name(sset->type_string(), sset->name());
      const Ioss::SideBlockContainer &sblocks = sset->get_side_blocks();
      adios2::Variable<int8_t> sblocks_var =
          adios_wrapper.InquireVariable<int8_t>(encoded_name);
      // adios2::Variable<char> sblocks_var =
      //     adios_wrapper.InquireVariable<char>(encoded_name);
      if (!sblocks_var) {
        std::string stringified_side_block_names = stringify_side_block_names(sblocks);
         //String size +1 to have space to save '\0' final character.
        adios_wrapper.DefineVariable<int8_t>(encoded_name, {number_proc, INT_MAX}, {rank, 0}, {1, stringified_side_block_names.size()+1});
        //adios_wrapper.DefineVariable<char>(encoded_name, {number_proc, INT_MAX}, {rank, 0}, {1, stringified_side_block_names.size()});
      }
      define_entity_internal(sblocks, role);
    }
  }

  void DatabaseIO::define_global_variables()
  {
    adios_wrapper.DefineAttribute<double>(Time_scale_factor, timeScaleFactor);
    adios_wrapper.DefineMetaVariable<double>(Time_meta);
    adios_wrapper.DefineVariable<unsigned long>(Processor_id_meta, {number_proc}, {rank}, {1});
    adios_wrapper.DefineVariable<unsigned long>(Processor_number_meta, {number_proc}, {rank}, {1});
  }

  //------------------------------------------------------------------------
  // TODO: Complete Region function
  int64_t DatabaseIO::put_field_internal(const Ioss::Region * /* region */,
                                         const Ioss::Field &field, void *data,
                                         size_t data_size) const
  {
    {
      Ioss::SerializeIO serializeIO__(this);

      Ioss::Field::RoleType role       = field.get_role();
      int                   num_to_get = field.verify(data_size);
      throw "Not impleemented";
      return num_to_get;
    }
  }

  // TODO: write actual code!
  // Returns byte size of integers stored on the database...
  int DatabaseIO::int_byte_size_db() const { return 4; }

  int64_t DatabaseIO::put_field_internal(const Ioss::NodeBlock *nb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return put_field_internal_t(nb, field, data, data_size);
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
  void DatabaseIO::put_data(void *data, const std::string &encoded_name) const
  {
    adios2::Variable<T> entities = adios_wrapper.InquireVariable<T>(encoded_name);
    if (entities) {
      std::cout<<"Writing " << encoded_name << std::endl;
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

  void DatabaseIO::put_meta_variables(const std::string &encoded_name, const Ioss::Field &field,
                                      const std::string &entity_type,
                                      const std::string &field_name) const
  {
    adios_wrapper.PutMetaVariable<int>(Role_meta, field.get_role(), encoded_name);
    bool        transformed_field = use_transformed_storage(field, entity_type, field_name);
    std::string var_type =
        transformed_field ? field.transformed_storage()->name() : field.raw_storage()->name();
    adios_wrapper.PutMetaVariable<std::string>(Var_type_meta, var_type, encoded_name);
  }

  template <typename T>
  int64_t DatabaseIO::put_field_internal_t(T entity, const Ioss::Field &field, void *data,
                                           size_t data_size) const
  {
    if (!data || !data_size) {
      return 0;
    }
    std::string        entity_type = entity->type_string();
    const std::string &field_name  = field.get_name();
    if (find_field_in_mapset(entity_type, field_name, Ignore_fields)) {
      return 0;
    }

    int num_to_get = field.verify(data_size);
    if (num_to_get > 0) {
      std::string encoded_name = encode_field_name({entity_type, entity->name(), field_name});
      switch (field.get_type()) {
      case Ioss::Field::BasicType::DOUBLE: put_data<double>(data, encoded_name); break;
      case Ioss::Field::BasicType::INT32: put_data<int32_t>(data, encoded_name); break;
      case Ioss::Field::BasicType::INT64: put_data<int64_t>(data, encoded_name); break;
      case Ioss::Field::BasicType::COMPLEX: put_data<Complex>(data, encoded_name); break;
      case Ioss::Field::BasicType::CHARACTER: put_data<char>(data, encoded_name); break;
      default: {
        std::ostringstream errmsg;
        errmsg << "INTERNAL ERROR: Invalid field type. "
               << "Something is wrong in the Ioad::DatabaseIO::put_field_internal_t() function. "
               << "Please report.\n";
        IOSS_ERROR(errmsg);
      }
      }
      put_meta_variables(encoded_name, field, entity_type, field_name);
    }
    return num_to_get;
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::ElementBlock *eb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return put_field_internal_t(eb, field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::EdgeBlock *eb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return put_field_internal_t(eb, field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::FaceBlock *fb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return put_field_internal_t(fb, field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::SideBlock *sb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return put_field_internal_t(sb, field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::NodeSet *ns, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return put_field_internal_t(ns, field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::EdgeSet *es, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return put_field_internal_t(es, field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::FaceSet *fs, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return put_field_internal_t(fs, field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::ElementSet *es, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return put_field_internal_t(es, field, data, data_size);
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::SideSet *ss, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return put_field_internal_t(ss, field, data, data_size);
    // int64_t res = put_field_internal_t(ss, field, data, data_size);
    // if (res) {
    //   std::string                   encoded_name = encode_sideblock_name(ss->type_string(),
    //   ss->name()); adios2::Variable<std::string> entities =
    //       adios_wrapper.InquireVariable<std::string>(encoded_name);
    //   std::vector<std::string> block_members;
    //   for (auto &sb : ss->get_side_blocks()) {
    //     block_members.push_back(sb->name());
    //   }
    //   if (entities && block_members.size()) {
    //     std::string stringified_block_members;
    //     for (std::string s : block_members) {
    //       stringified_block_members += "/" + s;
    //     }
    //     adios_wrapper.Put<std::string>(
    //         entities, stringified_block_members,
    //         adios2::Mode::Sync); // If not Sync, variables are not saved correctly.
    //   }
    // }
    // return res;
  }

  int64_t DatabaseIO::put_field_internal(const Ioss::CommSet *cs, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    // TODO: Make sure that Commset should be handled like a "set" and not like a "block".
    return put_field_internal_t(cs, field, data, data_size);
  }

  template <typename T> const std::string DatabaseIO::get_entity_type()
  {
    // Use "node" as default entity type to enable factory to create object.
    std::unique_ptr<T> entity(NewEntity<T>(this, "", "node", 0));
    return entity->type_string();
  }

  // NodeBlock has a different constructor...
  template <> const std::string DatabaseIO::get_entity_type<Ioss::NodeBlock>()
  {
    Ioss::NodeBlock nodeblock(this, "", 0, 1);
    return nodeblock.type_string();
  }

  // SideSet has a different constructor...
  template <> const std::string DatabaseIO::get_entity_type<Ioss::SideSet>()
  {
    Ioss::SideSet sideset(this, "");
    return sideset.type_string();
  }

  // SideBlock has a different constructor...
  template <> const std::string DatabaseIO::get_entity_type<Ioss::SideBlock>()
  {
    // default element arbitrarily set to "hex8": We need an element type
    // to be able to construct this sideblock.
    Ioss::SideBlock sideblock(this, "", "node", "hex8", 0);
    return sideblock.type_string();
  }

  template <typename T>
  DatabaseIO::FieldInfoType DatabaseIO::get_expected_variable_infos_from_map(
      const EntityMapType &entity_map, const std::string &entity_type,
      const std::string &block_name, const std::string &var_name) const
  {
    if (entity_map.find(block_name) == entity_map.end()) {
      std::ostringstream errmsg;
      errmsg << "ERROR: block name " << block_name << " not found.\n";
      IOSS_ERROR(errmsg);
    }

    if (entity_map.at(block_name).find(var_name) == entity_map.at(block_name).end()) {
      std::ostringstream errmsg;
      errmsg << "ERROR: block name not found or does not contain `" << var_name << "` field.\n";
      IOSS_ERROR(errmsg);
    }

    if (entity_map.at(block_name).at(var_name).second != adios2::helper::GetType<T>()) {
      std::ostringstream errmsg;
      errmsg << "ERROR: `" << var_name << "` field type does not match expected type.\n";
      IOSS_ERROR(errmsg);
    }
    std::string variable = encode_field_name({entity_type, block_name, var_name});
    return get_variable_infos<T>(variable);
  }

  DatabaseIO::FieldInfoType DatabaseIO::get_variable_infos_from_map(
      const EntityMapType &entity_map, const std::string &entity_type,
      const std::string &block_name, const std::string &var_name) const
  {
    // No check to verify that the block_name, entity_type, variable_name, and type exist as they
    // have been extracted earlier from the file and everything should still be up to date.
    std::string variable = encode_field_name({entity_type, block_name, var_name});
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
                "Ioad::DatabaseIO::get_variable_infos_from_map() function."
             << "Please report.\n";
      IOSS_ERROR(errmsg);
    }
    return FieldInfoType();
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

  template <>
  void DatabaseIO::get_entities<Ioss::NodeBlock>(const FieldsMapType &fields_map,
                                                 const FieldsMapType &properties_map)
  {
    const std::string block_type = get_entity_type<Ioss::NodeBlock>();
    // For exodusII, there is only a single node block which contains
    // all of the nodes.
    // The default id assigned is '1' and the name is 'nodeblock_1'
    const EntityMapType &entity_map = fields_map.at(block_type);
    std::string          block_name = "nodeblock_1";

    // `mesh_model_coordinates` field is automatically created in NodeBlock constructor.
    std::string coord_var_name = "mesh_model_coordinates";
    // Get `node_count` and `spatialDimension`.
    FieldInfoType model_coordinates_infos = get_expected_variable_infos_from_map<double>(
        entity_map, block_type, block_name, coord_var_name);
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
        FieldInfoType variable_infos =
            get_variable_infos_from_map(entity_map, block_type, block_name, variable.first);
        Ioss::Field field(variable.first, variable_infos.basic_type, variable_infos.variable_type,
                          variable_infos.role, variable_infos.node_boundaries_size);
        block->field_add(field);
        // TODO: Add offset?????
      }
    }
    add_entity_properties(block, properties_map);
    bool added = get_region()->add(block);
    if (!added) {
      delete block;
    }
  }

  template <typename T>
  void DatabaseIO::add_entity_property(Ioss::GroupingEntity *ge, const std::string &encoded_name,
                                       const std::string &property_name)
  {
    T variable;
    // Meta variable read with `Get()` instead of `GetMetaVariable()` because name is already
    // encoded. `GetMetaVariable()` simply encodes the variable name as a meta variable before
    // performing a `Get()` call.
    adios_wrapper.Get<T>(encoded_name, variable,
                         adios2::Mode::Sync); // If not Sync, variables are not saved correctly.
    Ioss::Property property(property_name, variable);
    ge->property_add(property);
  }

  void DatabaseIO::add_entity_properties(Ioss::GroupingEntity *ge,
                                         const FieldsMapType & properties_map, std::string name)
  {
    std::string entity_type = ge->type_string();
    if (properties_map.find(entity_type) == properties_map.end()) {
      return;
    }
    const EntityMapType &entity_property_map = properties_map.at(entity_type);
    GlobalMapType  properties_info;
    if (!name.empty()) {
      // Verify that the given name exists.
      if (entity_property_map.find(name) == entity_property_map.end()) {
        std::ostringstream errmsg;
        errmsg << "ERROR: Property map doesn't contain "<< name <<" inside " << entity_type << ".\n";
        IOSS_ERROR(errmsg);
      }
      properties_info = entity_property_map.at(name);
    }
    else {
      std::string entity_name = ge->name();
      if (entity_property_map.find(entity_name) == entity_property_map.end()) {
        return;
      }
      properties_info = entity_property_map.at(entity_name);
    }
    for (auto &property_info : properties_info) {
      std::string property_name = property_info.first;
      std::string encoded_name  = property_info.second.first;
      std::string type          = property_info.second.second;
      if (type == adios2::helper::GetType<std::string>()) {
        add_entity_property<std::string>(ge, encoded_name, property_name);
      }
      else if (type == adios2::helper::GetType<double>()) {
        add_entity_property<double>(ge, encoded_name, property_name);
      }
      else if (type == adios2::helper::GetType<int64_t>()) {
        add_entity_property<int64_t>(ge, encoded_name, property_name);
      }
      else {
        std::ostringstream errmsg;
        errmsg << "ERROR: Property type `" << type << "` not supported.\n";
        IOSS_ERROR(errmsg);
      }
    }
  }

  template<typename T>
  std::string DatabaseIO::get_property_value(const FieldsMapType & properties_map,
  const std::string &entity_type, const std::string &entity_name, const std::string &property_name) const
  {
    T property_value;
    if (properties_map.find(entity_type) == properties_map.end()) {
      return property_value;
    }
    const EntityMapType &entity_property_map = properties_map.at(entity_type);
    if (entity_property_map.find(entity_name) == entity_property_map.end()) {
      return property_value;
    }
    const GlobalMapType properties_info = entity_property_map.at(entity_name);
    const auto &property_info = properties_info.find(property_name);
    if(property_info != properties_info.end()) {
      const std::string encoded_name  = property_info->second.first;
      const std::string type          = property_info->second.second;
      const std::string expected_type = adios2::helper::GetType<T>();
      if(type != expected_type)
      {
        std::ostringstream errmsg;
        errmsg << "ERROR: Property type `" << type << "` doesn't match expected type `"<< expected_type << "`.\n";
        IOSS_ERROR(errmsg);
      }
      // Meta variable read with `Get()` instead of `GetMetaVariable()` because name is already
      // encoded. `GetMetaVariable()` simply encodes the variable name as a meta variable before
      // performing a `Get()` call.
      adios_wrapper.Get<T>(encoded_name, property_value,
                         adios2::Mode::Sync); // If not Sync, variables are not saved correctly.
    }
    return property_value;
  }



  template <>
  void DatabaseIO::get_entities<Ioss::SideSet>(const FieldsMapType &fields_map,
                                               const FieldsMapType &properties_map)
  {
    std::string entity_type = get_entity_type<Ioss::SideSet>();
    if (fields_map.find(entity_type) == fields_map.end()) {
      return;
    }
    std::string sideblock_type = get_entity_type<Ioss::SideBlock>();
    // Do SideBlock exist in the fields_map. This will be used when loading the SideBlock
    // for each SideSet, but we check this here outside of a loop as it only needs to be done once.
    bool no_sideblocks = (fields_map.find(sideblock_type) == fields_map.end());

    const EntityMapType &sidesets_map   = fields_map.at(entity_type);
    const EntityMapType &sideblocks_map = fields_map.at(sideblock_type);
    for (auto &entity : sidesets_map) {
      std::string    entity_name = entity.first;
      std::cout<<"Sideset:"<<entity_name<<std::endl;
      Ioss::SideSet *ss          = new Ioss::SideSet(this, entity_name);
      bool           ss_added    = get_region()->add(ss);
      add_entity_properties(ss, properties_map);
      if (!ss_added) {
        delete ss;
        return;
      }
      for (auto &variable_pair : entity.second) {
        // Since some fields are created automatically, we need to avoid recreating them when
        // loading the file.
        // First, check that field is actually a field and not a list of sideblocks
        if (!is_sideblock_name(variable_pair.first)) {
          std::cout<<"not sideblock:"<<variable_pair.first<<std::endl;
          if (!ss->field_exists(variable_pair.first)) {
            std::cout<<"sideset add field"<<std::endl;
            FieldInfoType infos = get_variable_infos_from_map(sidesets_map, entity_type,
                                                              entity_name, variable_pair.first);
            // TODO: Should we have a sanity check to verify that the field size is the same as
            // the entity size?
            Ioss::Field field(variable_pair.first, infos.basic_type, infos.variable_type,
                              infos.role, infos.node_boundaries_size);
            ss->field_add(field);
          }
        }
        else {
          std::cout<<"sideblock:"<<variable_pair.first<<std::endl;
          // The field in the bp file is actually a list of sideblocks
          // If no sideblock, don't worry about this section and move on to the next
          // sideset.
          if (no_sideblocks) {
            continue;
          }
          // adios2::Variable<char> sideblock_var =
          //     adios_wrapper.InquireVariable<char>(variable_pair.second.first);
          adios2::Variable<int8_t> sideblock_var =
              adios_wrapper.InquireVariable<int8_t>(variable_pair.second.first);

          if (sideblock_var) {
            // TODO: Remove this hardcoded size and manage variable (delete)
            BlockInfoType block_infos = get_block_infos<int8_t>(sideblock_var);
            char* stringified_names = new char[block_infos.Count[1]];
            get_data<int8_t>(static_cast<void*>(stringified_names), variable_pair.second.first);
            //get_data<char>(static_cast<void*>(stringified_names), variable_pair.second.first);
            std::vector<std::string> block_names = Ioss::tokenize(stringified_names, Sideblock_separator);
            delete []stringified_names;
            for (std::string block_name : block_names) {
              std::cout<<"sidebloick:" << block_name<<std::endl;
              if (sideblocks_map.find(block_name) != sideblocks_map.end()) {
                bool first = true;
                for (auto &sideblock_field_pair : sideblocks_map.at(block_name)) {
                  std::string   field_name  = sideblock_field_pair.first;
                  std::cout<<"sideblock field:"<<field_name<<std::endl;
                  FieldInfoType block_infos = get_variable_infos_from_map(
                      sideblocks_map, sideblock_type, block_name, field_name);
                  // Create sideblock first
                  if (first) {
                    Ioss::SideBlock *sideblock = new Ioss::SideBlock(
                        this, block_name, block_infos.topology, block_infos.parent_topology,
                        block_infos.node_boundaries_size);
                    bool block_added = ss->add(sideblock);
                    if (!block_added) {
                      delete sideblock;
                      std::ostringstream errmsg;
                      errmsg << "ERROR: Could not add sideblock `" << block_name << "` to sideset `"
                             << ss->name() << "`.\n";
                      IOSS_ERROR(errmsg);
                    }
                    first = false;
                  }
                  Ioss::SideBlock *sideblock = ss->get_side_block(block_name);
                  add_entity_properties(sideblock, properties_map);
                  // Add fields to sideblock if it was not automatically added.
                  if (!sideblock->field_exists(sideblock_field_pair.first)) {
                    Ioss::Field field(field_name, block_infos.basic_type, block_infos.variable_type,
                                      block_infos.role, block_infos.node_boundaries_size);
                    sideblock->field_add(field);
                    //                                  side_block->set_parent_element_block(block);
                  }
                }
              }
              else {
                // throw an error if sideblock not found? Check if sideblock name matches sideset
                // name: it could
                // have been added automatically at read time if the list was empty (see STK).
              }
            }
          }
        }
      }
    }
  }

  // Takes an extra unused parameter "entity_type" to match the API of the function
  // "NewEntity" used for objects that are not EntitySets which require that parameter.
  // Having matching APIs allows to call this function from generic templated functions
  // that do not have to be specialized to call this function with a different number of
  // parameters.
  template <typename T>
  auto DatabaseIO::NewEntity(DatabaseIO *io_database, const std::string &my_name,
                             const std::string & /*entity_type*/, size_t entity_count)
      -> IossHas3ParametersConstructor<T> *
  {
    return new T(io_database, my_name, entity_count);
  }

  template <typename T>
  auto DatabaseIO::NewEntity(DatabaseIO *io_database, const std::string &my_name,
                             const std::string &entity_type, size_t entity_count)
      -> IossHas4ParametersConstructor<T> *
  {
    return new T(io_database, my_name, entity_type, entity_count);
  }

  template <typename T>
  void DatabaseIO::get_entities(const FieldsMapType &fields_map,
                                const FieldsMapType &properties_map)
  {
    std::string entity_type = get_entity_type<T>();
    if (fields_map.find(entity_type) == fields_map.end()) {
      return;
    }
    const EntityMapType &entity_map = fields_map.at(entity_type);
    for (auto &variable_pair : entity_map) {
      std::string entity_name = variable_pair.first;
      // Get size and type info for the entity using the first element in the map.
      FieldInfoType infos_to_create_entity = get_variable_infos_from_map(
          entity_map, entity_type, entity_name, variable_pair.second.begin()->first);
      std::string block_type = get_property_value<std::string>(properties_map, entity_type, entity_name, "entity_type");
      block_type = block_type.empty() ? infos_to_create_entity.topology : block_type; 
      T *entity = NewEntity<T>(this, entity_name, block_type,
                               infos_to_create_entity.node_boundaries_size);
      add_entity_properties(entity, properties_map);

      bool added = get_region()->add(entity);
      if (!added) {
        delete entity;
      }
      for (auto &field_pair : variable_pair.second) {
        // Since some fields are created automatically, we need to avoid recreating them when
        // loading the file.
        // Note: We get the information about the first field twice: once before this loop, and
        // once inside the loop. The code could be modified to perform this action only once but
        // most likely the code will be more complex and will not save a lot of computation time.
        if (!entity->field_exists(field_pair.first)) {
          FieldInfoType infos =
              get_variable_infos_from_map(entity_map, entity_type, entity_name, field_pair.first);
          // TODO: Should we have a sanity check to verify that the field size is the same as
          // the entity size?
          Ioss::Field field(field_pair.first, infos.basic_type, infos.variable_type, infos.role,
                            infos.node_boundaries_size);
          entity->field_add(field);
        }
      }
    }
  }

  // TODO: Consolidate this code with generice `get_entities()`  function as there is very little difference
  // between the two.
  template <>
  void DatabaseIO::get_entities<Ioss::CommSet>(const FieldsMapType &fields_map,
                                const FieldsMapType &properties_map)
  {
    std::string entity_type = get_entity_type<Ioss::CommSet>();
    if (fields_map.find(entity_type) == fields_map.end()) {
      return;
    }
    const EntityMapType &entity_map = fields_map.at(entity_type);
    for (auto &variable_pair : entity_map) {
      std::string entity_name = variable_pair.first;
      // Get size and type info for the entity using the first element in the map.
      FieldInfoType infos_to_create_entity = get_variable_infos_from_map(
          entity_map, entity_type, entity_name, variable_pair.second.begin()->first);
      std::string block_type = get_property_value<std::string>(properties_map, entity_type, entity_name, "entity_type");
      std::string commset_name = "commset_" + block_type;

      block_type = block_type.empty() ? infos_to_create_entity.topology : block_type; 
      Ioss::CommSet *entity = NewEntity<Ioss::CommSet>(this, commset_name, block_type,
                               infos_to_create_entity.node_boundaries_size);
      add_entity_properties(entity, properties_map);
      // Save original name as entity property
      Ioss::Property property(original_name, entity_name);
      entity->property_add(property);

      bool added = get_region()->add(entity);
      if (!added) {
        delete entity;
      }
      for (auto &field_pair : variable_pair.second) {
        // Since some fields are created automatically, we need to avoid recreating them when
        // loading the file.
        // Note: We get the information about the first field twice: once before this loop, and
        // once inside the loop. The code could be modified to perform this action only once but
        // most likely the code will be more complex and will not save a lot of computation time.
        if (!entity->field_exists(field_pair.first)) {
          FieldInfoType infos =
              get_variable_infos_from_map(entity_map, entity_type, entity_name, field_pair.first);
          // TODO: Should we have a sanity check to verify that the field size is the same as
          // the entity size?
          Ioss::Field field(field_pair.first, infos.basic_type, infos.variable_type, infos.role,
                            infos.node_boundaries_size);
          entity->field_add(field);
        }
      }
    }
  }


  std::string DatabaseIO::get_optional_string_variable(const std::string &field_name,
                                                       const std::string &string_variable) const
  {
    std::vector<std::string> tokens = Ioss::tokenize(field_name, Name_separator);
    std::string              entity = encode_field_name({tokens[0], tokens[1]});
    auto                     v      = adios_wrapper.InquireVariable<std::string>(
        adios_wrapper.EncodeMetaVariable(string_variable, entity));
    if (v) {
      return adios_wrapper.GetMetaVariable<std::string>(string_variable, entity);
    }
    else {
      return std::string{};
    }
  }

  template<typename T>
  DatabaseIO::BlockInfoType DatabaseIO::get_block_infos(const adios2::Variable<T> &var) const
  {
    std::map<size_t, std::vector<typename adios2::Variable<T>::Info>> allblocks =
        adios_wrapper.AllStepsBlocksInfo(var);
    if (allblocks.empty()) {
      std::ostringstream errmsg;
      errmsg << "ERROR: Empty BP variable\n";
      IOSS_ERROR(errmsg);
    }
    BlockInfoType infos;
    for (auto &blockpair : allblocks) {
      std::vector<typename adios2::Variable<T>::Info> &blocks = blockpair.second;
      // Find in vector if this variable is defined for the current rank process. This means
      // that there is one block for which the rank encoded as the first value in the `Start` array
      // matches the current rank.
      for (auto &block : blocks) {
        if (block.Start[0] != rank) {
          // This is not the block corresponding to the current process (rank).
          continue;
        }
        if (blockpair.first == 0) {
          // infos.node_boundaries_start = block_node_boundaries_start;
          infos.Count = block.Count;
        }
        else if (infos.Count != block.Count) {
          std::ostringstream errmsg;
          errmsg
              << "ERROR: Variable changes sizes over steps. Not supported by Ioad::DatabaseIO.\n";
          IOSS_ERROR(errmsg);
        }
        // }
        // Steps: will save last step
        infos.steps.push_back(blockpair.first);
        // We found the block we were searching for, no need to continue looping over all
        // the other blocks.
        break;
      }
    }
    return infos;
  }

  template <typename T>
  DatabaseIO::FieldInfoType DatabaseIO::get_variable_infos(const std::string &var_name) const
  {
    FieldInfoType infos;
    adios2::Variable<T> var = adios_wrapper.InquireVariable<T>(var_name);

    // Dimension is expected to be 3 because:
    // 0 = rank
    // 1 = size of field
    // 2 = number of components
    size_t ndim = var.Shape().size();
    if (ndim != 3) {
      std::ostringstream errmsg;
      errmsg << "ERROR: BP variable dimension should be 3.\n";
      IOSS_ERROR(errmsg);
    }
    BlockInfoType block_infos = get_block_infos<T>(var);
    size_t laststep = 0;
    if(!block_infos.steps.empty()) {
      laststep = block_infos.steps.back();
      // Skipping dimension=0 which is equal to the rank. Dimension=1 encodes beginning and count
      // of node count for this variable. Dimension=2 encodes the number of components for this
      // variables, and should always start at 0;
      infos.component_count      = block_infos.Count[2];
      infos.node_boundaries_size = block_infos.Count[1];
      infos.steps                = block_infos.steps;
    }
    // Get topology: Only relevant for blocks, not for sets.
    infos.topology = get_optional_string_variable(var_name, Topology_meta);
    // Get parent topology: Only relevant for sideblocks.
    infos.parent_topology = get_optional_string_variable(var_name, Parent_topology_meta);
    // Get VariableType
    infos.variable_type = adios_wrapper.GetMetaVariable<std::string>(Var_type_meta, var_name);
    infos.basic_type = template_to_basic_type<T>();
    // Only transient fields can have steps.
    infos.role =
        static_cast<Ioss::Field::RoleType>(adios_wrapper.GetMetaVariable<int>(Role_meta, var_name));
    if (infos.role != Ioss::Field::RoleType::TRANSIENT && laststep != 0) {
      std::ostringstream errmsg;
      errmsg << "ERROR: Last step should be 0 for non-transient fields. "
             << "Something is wrong in the Ioad::DatabaseIO::get_variable_size() function.\n";
      IOSS_ERROR(errmsg);
    }
    return infos;
  }

  void DatabaseIO::get_globals(const GlobalMapType &globals_map, const FieldsMapType &properties_map)
  {
    std::cout<<"get_globals"<<std::endl;
    // Check "time" attribute and global variable.
    adios2::Attribute<double> timeScaleFactor_attr =
        adios_wrapper.InquireAttribute<double>(Time_scale_factor);
    if (timeScaleFactor_attr) {
      timeScaleFactor = get_attribute<double>(Time_scale_factor);
    }
    else {
      timeScaleFactor = 1.0;
    }

    Ioss::SerializeIO serializeIO__(this);
    Ioss::Region *    this_region = get_region();
    if (globals_map.find(Time_meta) != globals_map.end()) {
      // Load time steps
      // 1) Check that the time type is `double` as expected.
      adios2::Variable<double> time_var = adios_wrapper.InquireVariable<double>(Time_meta);
      if (time_var) {
        std::vector<double>      tsteps(time_var.Steps());
        std::cout<<"Steps:" << time_var.Steps() << std::endl;
        if(!is_streaming) {
          time_var.SetStepSelection(std::make_pair(time_var.StepsStart(), time_var.Steps())); // Doesn't work with streaming.
        }
        else {
          // For streaming, we probably only want to read the current time as we do not want to 
          // use SetStepSelection that would prohibit the usage of `BeginStep()/EndStep()`.
          // std::ostringstream errmsg;
          // errmsg << "ERROR: Streaming is not yet supported for reading.\n";
          // IOSS_ERROR(errmsg);
        }
        adios_wrapper.Get(time_var, tsteps.data(), adios2::Mode::Sync);
        for (size_t step = 0; step < time_var.Steps(); step++) {
          std::cout<<"Step:"<<step<<std::endl;
          // if (tsteps[i] <= last_time) { TODO: Check last time step before writing everything
          this_region->add_state__(tsteps[step] * timeScaleFactor);
        }
      }
      else {
        std::ostringstream errmsg;
        errmsg << "ERROR: Timestep global detected in file but cannot read it.\n";
        IOSS_ERROR(errmsg);
      }
    }
    // Get Processor information
    get_data<unsigned long>(static_cast<void*>(&proc_info.processor_id), Processor_id_meta);
    get_data<unsigned long>(static_cast<void*>(&proc_info.processor_number), Processor_number_meta);

    // adios2::Variable<unsigned long> processor_id_var =
    //     adios_wrapper.InquireVariable<unsigned long>(Processor_id_meta);
    // if (processor_id_var) {
    //   processor_id_var.SetSelection(adios2::Box<adios2::Dims>({rank}, {1}));
    //   adios_wrapper.Get<unsigned long>(processor_id_var, proc_info.processor_id,
    //                        adios2::Mode::Sync); // If not Sync, variables are not saved correctly.
    // }
    // else {
    //     std::ostringstream errmsg;
    //     errmsg << "ERROR: Required `"<< Processor_id_meta <<"` variable not found in file.\n";
    //     IOSS_ERROR(errmsg);
    // }
    // Add region properties
    add_entity_properties(this_region, properties_map, region_name);
  }

  void DatabaseIO::read_meta_data__()
  {
    std::cout<<"read_meta_data__"<<std::endl;
    // Only get schema version attribute as it is the only one we expect.
    get_attribute<unsigned int>(Schema_version_string);

    // Get all variables
    GlobalMapType globals_map;
    // Entity_type/Entity_name::Property -> name & Type
    FieldsMapType properties_map;
    // Entity_type/Entity_name/Field -> name & Type
    FieldsMapType fields_map;

    const std::map<std::string, std::map<std::string, std::string>> variables =
        adios_wrapper.AvailableVariables();
    for (const auto &vpair : variables) {
      const std::string &      name           = vpair.first;
      auto                     name_type_pair = std::make_pair(name, vpair.second.at("Type"));
      std::vector<std::string> tokens         = Ioss::tokenize(name, Name_separator);
      switch (tokens.size()) {
      case 1: globals_map[tokens[0]] = name_type_pair; break;
      case 2: {
        // Only save properties.
        std::string meta_var         = "";
        std::string property         = "";
        std::tie(meta_var, property) = adios_wrapper.DecodeMetaName(tokens[1]);
        if (!property.compare(0, property_meta.size(), property_meta)) {
          properties_map[tokens[0]][meta_var][property.substr(property_meta.size())] =
              name_type_pair;
        }
        break;
      }
      case 3: {
        std::string is_meta            = "";
        std::tie(std::ignore, is_meta) = adios_wrapper.DecodeMetaName(tokens[2]);
        if (is_meta.empty()) {
          fields_map[tokens[0]][tokens[1]][tokens[2]] = name_type_pair;
        }
        break;
      }
      default: {
        std::ostringstream errmsg;
        errmsg << "ERROR: Invalid encoded entity name.\n";
        IOSS_ERROR(errmsg);
      }
      }
    }

    // read_region(reader_io);
    // read_communication_metadata();

    get_globals(globals_map, properties_map);
    get_entities<Ioss::NodeBlock>(fields_map, properties_map);
    get_entities<Ioss::EdgeBlock>(fields_map, properties_map);
    get_entities<Ioss::FaceBlock>(fields_map, properties_map);
    get_entities<Ioss::ElementBlock>(fields_map, properties_map);

    check_side_topology();

    get_entities<Ioss::SideSet>(fields_map, properties_map);
    get_entities<Ioss::NodeSet>(fields_map, properties_map);
    get_entities<Ioss::EdgeSet>(fields_map, properties_map);
    get_entities<Ioss::FaceSet>(fields_map, properties_map);
    get_entities<Ioss::ElementSet>(fields_map, properties_map);
    get_entities<Ioss::CommSet>(fields_map, properties_map);

// Ioss::Region *    region = get_region();
//     region->property_add(Ioss::Property("global_node_count", global_nodes));
//     region->property_add(Ioss::Property("global_element_count", global_elements));
//     region->property_add(Ioss::Property("global_element_block_count", global_eblocks));
//     region->property_add(Ioss::Property("global_node_set_count", global_nsets));
//     region->property_add(Ioss::Property("global_side_set_count", global_ssets));

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
    return get_field_internal_t(nb, field, data, data_size);
  }
  int64_t DatabaseIO::get_field_internal(const Ioss::EdgeBlock *eb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return get_field_internal_t(eb, field, data, data_size);
  }
  int64_t DatabaseIO::get_field_internal(const Ioss::FaceBlock *fb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return get_field_internal_t(fb, field, data, data_size);
  }
  int64_t DatabaseIO::get_field_internal(const Ioss::ElementBlock *eb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return get_field_internal_t(eb, field, data, data_size);
  }
  int64_t DatabaseIO::get_field_internal(const Ioss::SideBlock *sb, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return get_field_internal_t(sb, field, data, data_size);
  }
  int64_t DatabaseIO::get_field_internal(const Ioss::NodeSet *ns, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return get_field_internal_t(ns, field, data, data_size);
  }
  int64_t DatabaseIO::get_field_internal(const Ioss::EdgeSet *es, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return get_field_internal_t(es, field, data, data_size);
  }
  int64_t DatabaseIO::get_field_internal(const Ioss::FaceSet *fs, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return get_field_internal_t(fs, field, data, data_size);
  }
  int64_t DatabaseIO::get_field_internal(const Ioss::ElementSet *es, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return get_field_internal_t(es, field, data, data_size);
  }
  int64_t DatabaseIO::get_field_internal(const Ioss::SideSet *ss, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return get_field_internal_t(ss, field, data, data_size);

    // if (res) {
    //   std::string                   encoded_name = encode_sideblock_name(ss->type_string(),
    //   ss->name()); adios2::Variable<std::string> entities =
    //       adios_wrapper.InquireVariable<std::string>(encoded_name);
    //   std::vector<std::string> block_members;
    //   for (auto &sb : ss->get_side_blocks()) {
    //     block_members.push_back(sb->name());
    //   }
    //   if (entities && block_members.size()) {
    //     std::string stringified_block_members;
    //     for (std::string s : block_members) {
    //       stringified_block_members += "/" + s;
    //     }
    //     adios_wrapper.Put<std::string>(
    //         entities, stringified_block_members,
    //         adios2::Mode::Sync); // If not Sync, variables are not saved correctly.
    //   }
    // }
    // return res;
  }
  int64_t DatabaseIO::get_field_internal(const Ioss::CommSet *cs, const Ioss::Field &field,
                                         void *data, size_t data_size) const
  {
    return get_field_internal_t(cs, field, data, data_size);
  }

  int64_t DatabaseIO::get_field_internal_t(const Ioss::GroupingEntity *entity,
                                           const Ioss::Field &field, void *data,
                                           size_t data_size) const
  {
    if (!data || !data_size) {
      return 0;
    }
    int num_to_get = field.verify(data_size);
    if (num_to_get > 0) {
      const std::string  entity_type = entity->type_string();

      // Check if field name has changed. Rely on property `original_name` if
      // it exists.
      const std::string entity_name = entity->property_exists(original_name)
                                          ? entity->get_property(original_name).get_string()
                                          : entity->name();
      const std::string &field_name = field.get_name();

      std::string encoded_name = encode_field_name({entity_type, entity_name, field_name});
      bool use_step_selection = false;
      if(field.get_role() == Ioss::Field::RoleType::TRANSIENT && !is_streaming)
      {
        use_step_selection = true;
      }
      switch (field.get_type()) {
      case Ioss::Field::BasicType::DOUBLE:
        get_data<double>(data, encoded_name, use_step_selection);
        break;
      case Ioss::Field::BasicType::INT32:
        get_data<int32_t>(data, encoded_name, use_step_selection);
        break;
      case Ioss::Field::BasicType::INT64:
        get_data<int64_t>(data, encoded_name, use_step_selection);
        break;
      case Ioss::Field::BasicType::COMPLEX:
        get_data<Complex>(data, encoded_name, use_step_selection);
        break;
      case Ioss::Field::BasicType::CHARACTER:
        get_data<char>(data, encoded_name, use_step_selection);
        break;
      default:
        std::ostringstream errmsg;
        errmsg << "INTERNAL ERROR: Invalid field type. "
               << "Something is wrong in the Ioad::DatabaseIO::get_field_internal_t() function. "
               << "Please report.\n";
        IOSS_ERROR(errmsg);
      }
    }
    return num_to_get;
  }

  template <typename T>
  void DatabaseIO::get_data(void *data, const std::string &encoded_name, bool use_step_selection) const
  {
    std::cout<<"get data:"<<encoded_name<<std::endl;
    adios2::Variable<T> entities   = adios_wrapper.InquireVariable<T>(encoded_name);
    if (entities) {
      T *rdata = static_cast<T *>(data);
      adios2::Dims size = entities.Shape();
      size[0]=1;
      adios2::Dims offset = entities.Start();
      offset[0]=rank;

      entities.SetSelection(adios2::Box<adios2::Dims>(offset, size));
      //if transient, set step that should be read if not streaming.
      if(use_step_selection) {
          size_t step = get_current_state();
          entities.SetStepSelection(std::make_pair(step, 1));
      }
      else {
          // if streaming, we are reading step selected by `BeginStep()/EndStep()`.
      }

      // TODO: Set selection per rank. Support written by N processes, and loaded by M processes.
      adios_wrapper.Get<T>(entities, rdata,
                           adios2::Mode::Sync); // If not Sync, variables are not saved correctly.
    }
    else {
      if (!is_streaming) {
        std::ostringstream errmsg;
        errmsg << "ERROR: Required `" << encoded_name << "` variable not found in file.\n";
        IOSS_ERROR(errmsg);
      }
      else {
        IOSS_WARNING << "WARNING: The variable `" << encoded_name << "` was not found.\n";
      }
    }
  }

  void DatabaseIO::compute_block_membership__(Ioss::SideBlock *         efblock,
                                              std::vector<std::string> &block_membership) const
  {
    const Ioss::ElementBlockContainer &element_blocks = get_region()->get_element_blocks();
    assert(Ioss::Utils::check_block_order(element_blocks));

    Ioss::Int64Vector block_ids(element_blocks.size());
    if (block_ids.size() == 1) {
      block_ids[0] = 1;
    }
    else {
      Ioss::Int64Vector element_side;
      if (int_byte_size_api() == 4) {
        Ioss::IntVector es32;
        efblock->get_field_data("element_side", es32);
        element_side.resize(es32.size());
        std::copy(es32.begin(), es32.end(), element_side.begin());
      }
      else {
        efblock->get_field_data("element_side", element_side);
      }

      size_t              number_sides = element_side.size() / 2;
      Ioss::ElementBlock *block        = nullptr;
      for (size_t iel = 0; iel < number_sides; iel++) {
        int64_t elem_id = element_side[2 * iel]; // Vector contains both element and side.
        // elem_id         = elemMap.global_to_local(elem_id);
        if (block == nullptr || !block->contains(elem_id)) {
          block = get_region()->get_element_block(elem_id);
          assert(block != nullptr);
          size_t block_order = block->get_property("original_block_order").get_int();
          assert(block_order < block_ids.size());
          block_ids[block_order] = 1;
        }
      }
    }

    // Synchronize among all processors....
    // if (isParallel) {
    //   util().global_array_minmax(block_ids, Ioss::ParallelUtils::DO_MAX);
    // }

    for (const auto block : element_blocks) {
      size_t block_order = block->get_property("original_block_order").get_int();
      assert(block_order < block_ids.size());
      if (block_ids[block_order] == 1) {
        if (!Ioss::Utils::block_is_omitted(block)) {
          block_membership.push_back(block->name());
        }
      }
    }
  }

  int DatabaseIO::get_current_state() const
  {
    // value returned is 1-based, whereas ADIOS expect 0-based values
    int step = get_region()->get_current_state() - 1;

    if (step < 0) {
      std::ostringstream errmsg;
      errmsg << "ERROR: No currently active state.  The calling code must call "
                "Ioss::Region::begin_state(int step)\n"
             << "       to set the database timestep from which to read the transient data.\n"
             << "       [" << get_filename() << "]\n";
      IOSS_ERROR(errmsg);
    }
    return step;
  }

} // namespace Ioad

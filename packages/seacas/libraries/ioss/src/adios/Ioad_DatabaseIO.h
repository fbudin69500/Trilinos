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

// -*- Mode: c++ -*-
#ifndef IOSS_Ioad_DatabaseIO_h
#define IOSS_Ioad_DatabaseIO_h

#include <Ioss_DBUsage.h>
#include <Ioss_DatabaseIO.h>
#include "Ioss_EntitySet.h"

#include "Ioss_SideSet.h"         // for SideBlockContainer, SideSet

#include "Ioss_Field.h" // for Field, etc
#include <AdiosWrapper.h>
#include <set>

namespace Ioss {
  class GroupingEntity;
  class Region;
  class EntityBlock;
  class NodeBlock;
  class SideBlock;
  class ElementBlock;
  class NodeSet;
  class SideSet;
  class CommSet;
} // namespace Ioss

/** \brief A namespace for the adios database format.
 */
namespace Ioad {

  class DatabaseIO : public Ioss::DatabaseIO
  {
  public:
    DatabaseIO(Ioss::Region *region, const std::string &filename, Ioss::DatabaseUsage db_usage,
               MPI_Comm communicator, const Ioss::PropertyManager &props);
    ~DatabaseIO();
    DatabaseIO(const DatabaseIO &from) = delete;
    DatabaseIO &operator=(const DatabaseIO &from) = delete;

    bool begin__(Ioss::State state) override;
    bool end__(Ioss::State state) override;

    unsigned int entity_field_support() const { return 0; }
    int          int_byte_size_db() const;

  private:
    int64_t get_field_internal(const Ioss::Region *reg, const Ioss::Field &field, void *data,
                               size_t data_size) const override;
    int64_t get_field_internal(const Ioss::NodeBlock *nb, const Ioss::Field &field, void *data,
                               size_t data_size) const override;
    int64_t get_field_internal(const Ioss::EdgeBlock *eb, const Ioss::Field &field, void *data,
                               size_t data_size) const override;
    int64_t get_field_internal(const Ioss::FaceBlock *fb, const Ioss::Field &field, void *data,
                               size_t data_size) const override;
    int64_t get_field_internal(const Ioss::ElementBlock *eb, const Ioss::Field &field, void *data,
                               size_t data_size) const override;
    int64_t get_field_internal(const Ioss::StructuredBlock *sb, const Ioss::Field &field,
                               void *data, size_t data_size) const override
    {
      return -1;
    }
    int64_t get_field_internal(const Ioss::SideBlock *sb, const Ioss::Field &field, void *data,
                               size_t data_size) const override;
    int64_t get_field_internal(const Ioss::NodeSet *ns, const Ioss::Field &field, void *data,
                               size_t data_size) const override;
    int64_t get_field_internal(const Ioss::EdgeSet *es, const Ioss::Field &field, void *data,
                               size_t data_size) const override;
    int64_t get_field_internal(const Ioss::FaceSet *fs, const Ioss::Field &field, void *data,
                               size_t data_size) const override;
    int64_t get_field_internal(const Ioss::ElementSet *es, const Ioss::Field &field, void *data,
                               size_t data_size) const override;
    int64_t get_field_internal(const Ioss::SideSet *ss, const Ioss::Field &field, void *data,
                               size_t data_size) const override;
    int64_t get_field_internal(const Ioss::CommSet *cs, const Ioss::Field &field, void *data,
                               size_t data_size) const override;

    int64_t get_field_internal_t(const Ioss::GroupingEntity *entity,
                               const Ioss::Field &field, void *data, size_t data_size) const;
    template <typename T>
    void get_data(void *data, const std::string &encoded_name) const;

    int64_t put_field_internal(const Ioss::Region *reg, const Ioss::Field &field, void *data,
                               size_t data_size) const override;
    int64_t put_field_internal(const Ioss::NodeBlock *nb, const Ioss::Field &field, void *data,
                               size_t data_size) const override;
    int64_t put_field_internal(const Ioss::EdgeBlock *eb, const Ioss::Field &field, void *data,
                               size_t data_size) const override;
    int64_t put_field_internal(const Ioss::FaceBlock *fb, const Ioss::Field &field, void *data,
                               size_t data_size) const override;
    int64_t put_field_internal(const Ioss::ElementBlock *eb, const Ioss::Field &field, void *data,
                               size_t data_size) const override;
    int64_t put_field_internal(const Ioss::SideBlock *sb, const Ioss::Field &field, void *data,
                               size_t data_size) const override;
    int64_t put_field_internal(const Ioss::NodeSet *ns, const Ioss::Field &field, void *data,
                               size_t data_size) const override;
    int64_t put_field_internal(const Ioss::EdgeSet *es, const Ioss::Field &field, void *data,
                               size_t data_size) const override;
    int64_t put_field_internal(const Ioss::FaceSet *fs, const Ioss::Field &field, void *data,
                               size_t data_size) const override;
    int64_t put_field_internal(const Ioss::ElementSet *es, const Ioss::Field &field, void *data,
                               size_t data_size) const override;
    int64_t put_field_internal(const Ioss::SideSet *ss, const Ioss::Field &field, void *data,
                               size_t data_size) const override;
    int64_t put_field_internal(const Ioss::CommSet *cs, const Ioss::Field &field, void *data,
                               size_t data_size) const override;
    int64_t put_field_internal(const Ioss::StructuredBlock *sb, const Ioss::Field &field,
                               void *data, size_t data_size) const override
    {
      return -1;
    }
    template<typename T>
    int64_t put_field_internal_t(T entity,
                               const Ioss::Field &field, void *data, size_t data_size) const;
    void    read_meta_data__() override;
    void    define_model(Ioss::Field::RoleType *role = nullptr);
    // Model definition that should not be re-defined when defining transient variables.
    void    define_global_variables();
    template<typename T>  T get_attribute(const std::string &attribute_name);
    // void read_region(adios2::IO & bpio);

    int64_t element_global_to_local__(int64_t global) const { return 0; }
    int64_t node_global_to_local__(int64_t global, bool must_exist) const { return 0; }

    template <typename T>
    void put_data(void *data, const std::string &encoded_name) const;
    template <typename T, typename = typename std::enable_if<!std::is_base_of<Ioss::EntitySet, T>::value, T>::type >
    void put_var_type(const Ioss::Field &field, const std::string &encoded_name,
                      bool transformed_field) const;
    template <typename T>
    void define_model_internal(const Ioss::Field &field, const std::string &encoded_name,
                               const std::string &entity_type, const std::string &field_name);
    template <typename T>
    void define_entity_internal(const T &entity_blocks, Ioss::Field::RoleType *role);

    bool use_transformed_storage(const Ioss::Field &field, const std::string &entity_type,
                                 const std::string &field_name) const;

    struct BlockInfoType
    {
      std::vector<size_t> steps;
      adios2::Dims Count;
    };

    struct FieldInfoType
    {
      // Information contained in block infos.
      std::vector<size_t> steps;
      size_t              node_boundaries_size = 0;
      size_t              component_count      = 0;
      // Contained in metavariables
      Ioss::Field::RoleType  role;
      std::string            variable_type;
      Ioss::Field::BasicType basic_type;
      std::string            topology;
      std::string            parent_topology;
    };

    struct ProcessorInfo
    {
      unsigned long processor_id = -1;
      unsigned long processor_number = -1;
    };
    template<typename T>
    BlockInfoType get_block_infos(const adios2::Variable<T> &var) const;

    template <typename T> FieldInfoType get_variable_infos(const std::string &var_name) const;
    using GlobalMapType = std::map<std::string, std::pair<std::string, std::string>>;
    using EntityMapType = std::map<std::string, GlobalMapType>;
    using FieldsMapType = std::map<std::string, EntityMapType>;

    template<typename T>
    std::string get_property_value(const FieldsMapType & properties_map, const std::string &entity_type, const std::string &entity_name, const std::string &property_name) const;

    template <typename T>
    FieldInfoType get_expected_variable_infos_from_map(const EntityMapType &fields_map,
                                                       const std::string &  entity_type,
                                                       const std::string &  entity_name,
                                                       const std::string &  var_name) const;
    FieldInfoType get_variable_infos_from_map(const EntityMapType &fields_map,
                                              const std::string &  entity_type,
                                              const std::string &  entity_name,
                                              const std::string &  var_name) const;

    std::string encode_field_name(std::vector<std::string> names) const;
    std::string encode_sideblock_name(std::string type_string, std::string name) const;
    bool        is_sideblock_name(std::string name) const;

    template <typename T>
    using IsIossEntityBlock =
        typename std::enable_if<std::is_base_of<Ioss::EntityBlock, T>::value>::type;
    template <typename T>
    using IsNotIossEntityBlock =
        typename std::enable_if<!std::is_base_of<Ioss::EntityBlock, T>::value>::type;
    template <typename T>
    using CompareEntityBlock =
        typename std::conditional<std::is_base_of<Ioss::EntityBlock, T>::value, Ioss::EntityBlock,
                                  Ioss::GroupingEntity>::type;

    template <typename T, typename = IsIossEntityBlock<T>>
    void define_entity_meta_variables(const std::string &encoded_name);

    template <typename T, typename = IsNotIossEntityBlock<T>, typename = void>
    void define_entity_meta_variables(const std::string &encoded_name);

    void define_field_meta_variables(const std::string &);

    void put_meta_variables(const std::string &encoded_name, const Ioss::Field &field,
                            const std::string &entity_type, const std::string &field_name) const;
    void write_meta_data();

    template <typename T>
    void add_entity_property(Ioss::GroupingEntity *ge, const std::string &encoded_name,
                             const std::string &var_name);
    void add_entity_properties(Ioss::GroupingEntity *ge, const FieldsMapType &properties_map, std::string name = "");

    void write_properties(const Ioss::GroupingEntity * const entity, const std::string & encoded_name);

    std::string stringify_side_block_names(const Ioss::SideBlockContainer &sblocks) const;

    template <typename T> void write_meta_data_container(const T &entity_blocks);

    template <typename T> const std::string get_entity_type();
    template <typename T> void              get_entities(const FieldsMapType &fields_map, const FieldsMapType &properties_map);
    std::string get_optional_string_variable(const std::string &field_name,
                                             const std::string &string_variable) const;

    template <typename T>
    using DerivedFromIossGroupingEntity =
        typename std::enable_if<std::is_base_of<Ioss::GroupingEntity, T>::value, bool>::type;

    template <typename T>
    using IossHas3ParametersConstructor =
        decltype(DerivedFromIossGroupingEntity<T>{}, T((DatabaseIO *){}, std::string{}, int64_t{}));

    template <typename T>
    using IossHas4ParametersConstructor =
        decltype(DerivedFromIossGroupingEntity<T>{},
                 T((DatabaseIO *){}, std::string{}, std::string{}, int64_t{}));

    template <typename T>
    auto NewEntity(DatabaseIO *io_database, const std::string &my_name,
                   const std::string & /*entity_type*/, size_t entity_count)
        -> IossHas3ParametersConstructor<T> *;

    template <typename T>
    auto NewEntity(DatabaseIO *io_database, const std::string &my_name,
                   const std::string &entity_type, size_t entity_count)
        -> IossHas4ParametersConstructor<T> *;

    void        get_globals(const GlobalMapType &globals_map, const FieldsMapType &properties_map);
    void        compute_block_membership__(Ioss::SideBlock *         efblock,
                                           std::vector<std::string> &block_membership) const;
    void        define_properties(const Ioss::GroupingEntity * entity_block,
                                  const std::string &               encoded_entity_name);
    std::string get_property_variable_name(const std::string &property_name);
    std::vector<std::string> properties_to_save(const Ioss::GroupingEntity *const entity_block);

    int  RankInit();
    bool begin_state__(Ioss::Region * /* region */, int state, double time);
    bool end_state__(Ioss::Region * /*region*/, int state, double time);

    int find_field_in_mapset(const std::string &entity_type, const std::string &field_name,
                             const std::map<std::string, std::set<std::string>> &mapset) const;
    unsigned long rank; // rank needs to be declared first to be initialized before adios_wrapper.
    mutable AdiosWrapper adios_wrapper; // adios_wrapper needs to be declared before bpio
                                        // and bp_engine to be initialized first.
    int           spatialDimension{0};
    unsigned long number_proc;

    ProcessorInfo proc_info;
  };
} // namespace Ioad
#endif

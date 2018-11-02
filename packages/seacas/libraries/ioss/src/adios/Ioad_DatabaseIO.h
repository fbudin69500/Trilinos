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

#include "Ioss_Field.h" // for Field, etc
#include <adios2.h>
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

  struct BlockInfoType
  {
    std::vector<size_t>    steps;
    size_t                 node_boundaries_start;
    size_t                 node_boundaries_size;
    size_t                 component_count = {0};
    Ioss::Field::RoleType  role;
    std::string            variable_type;
    Ioss::Field::BasicType basic_type;
  };

  class DatabaseIO : public Ioss::DatabaseIO
  {
  public:
    DatabaseIO(Ioss::Region *region, const std::string &filename, Ioss::DatabaseUsage db_usage,
               MPI_Comm communicator, const Ioss::PropertyManager &properties_x);
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
                               void *data, size_t data_size) const override;
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

    int64_t get_field_internal(const std::string &entity_type, const std::string &entity_name,
                               const Ioss::Field &field, void *data, size_t data_size) const;
    template <typename T>
    int64_t get_data(const Ioss::Field &field, void *data,
                     const std::string &encoded_name, size_t data_size) const;

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
      return 0;
    }
    int64_t put_field_internal(const std::string &entity_type, const std::string &entity_name,
                               const Ioss::Field &field, void *data, size_t data_size) const;
    void    read_meta_data__() override;
    void    define_model(Ioss::Field::RoleType *role = nullptr);
    // void read_region(adios2::IO & bpio);

    int64_t element_global_to_local__(int64_t global) const { return 0; }
    int64_t node_global_to_local__(int64_t global, bool must_exist) const { return 0; }

    template <typename T>
    int64_t put_data(const Ioss::Field &field, void *data,
                     const std::string &encoded_name, bool transformed_field,
                     size_t data_size) const;
    template <typename T>
    void                       define_model_internal(const Ioss::Field &field,
                                                     const std::string &encoded_name);
    template <typename T> void define_entity_internal(const T &entity_blocks, Ioss::Field::RoleType *role);

    bool use_transformed_storage(const Ioss::Field &field, const std::string &entity_type,
                                 const std::string &field_name) const;

    template <typename T>
    BlockInfoType get_variable_infos(const std::string &var_name) const;
    using EntityMapType =
        std::map<std::string, std::map<std::string, std::pair<std::string, std::string>>>;
    using VariableMapType = std::map<std::string, EntityMapType>;

    template <typename T>
    BlockInfoType get_variable_infos_from_map(const EntityMapType &variables_map,
                                              const std::string &entity_name,
                                              const std::string &block_name,
                                              const std::string &var_name) const;
    BlockInfoType get_variable_infos_from_map_no_check(const EntityMapType &variables_map,
                                                       const std::string &  entity_name,
                                                       const std::string &  block_name,
                                                       const std::string &  var_name) const;

    std::string encode_field_name(const std::string &entity_type, const std::string &entity_name,
                                  const std::string &field_name) const;
    std::tuple<std::string, std::string, std::string>
    decode_field_name(const std::string &encoded_name) const;

    void get_nodeblocks(const VariableMapType &variables);

class ADIOSWrapper: private adios2::ADIOS, private adios2::IO, private adios2::Engine
{
public:
  ADIOSWrapper(MPI_Comm communicator, const std::string &filename, bool is_input, unsigned long rank);
  ADIOSWrapper(ADIOSWrapper&& wrapper);
  ~ADIOSWrapper();
  void            BeginStep();
  void            EndStep();
  template <typename T>
  void DefineMetaVariable(const std::string &meta_name, const std::string &variable_name="");
  template <typename T>
  void PutMetaVariable(const std::string &meta_name, T value, const std::string &variable_name="");
  template <typename T>
  T GetMetaVariable(const std::string &meta_name, const std::string &variable_name="");
  std::pair<std::string, std::string> DecodeMetaName(std::string name) const;

  using adios2::IO::DefineVariable;
  using adios2::IO::DefineAttribute;
  using adios2::IO::InquireAttribute;
  using adios2::IO::InquireVariable;
  using adios2::IO::AvailableVariables;
  using adios2::Engine::Put;
  using adios2::Engine::Get;
  using adios2::Engine::AllStepsBlocksInfo;
private:
  adios2::IO     IOInit();
  adios2::Engine EngineInit(MPI_Comm communicator, const std::string &filename, bool is_input);
  std::string    EncodeMetaVariable(const std::string &meta_name, const std::string &variable_name="") const;

  const std::string m_MetaSeparator{"::"};

  const int                            m_Rank;
//  adios2::ADIOS *m_Adios;
  const MPI_Comm                       m_Communicator;

  //adios2::IO             m_BPIO;
//  mutable adios2::Engine m_BPEngine;
  bool                   m_OpenStep;
};

    // void add_attribute_fields(Ioss::GroupingEntity *block, std::vector<std::pair<size_t, size_t>>
    // size,
    //                      std::map<std::string, std::pair<std::string, std::string>>
    //                      field_names);
    int RankInit();
    bool begin_state__(Ioss::Region * /* region */, int state, double time);
    bool end_state__(Ioss::Region * /*region*/, int state, double time);

    int find_field_in_mapset(const std::string &entity_type, const std::string &field_name,
                             const std::map<std::string, std::set<std::string>> &mapset) const;
    unsigned long rank; // rank needs to be declared first to be initialized before ad_wrapper.
    mutable ADIOSWrapper ad_wrapper; // ad_wrapper needs to be declared before bpio and bp_engine to be initialized first.
    const std::string                                  schema_version_string = "IOSS_adios_version";
    int                                                spatialDimension{0};
    unsigned long                                      number_proc;
    const std::string                                  name_separator              = "/";
    const std::string                                  role_meta              = "role";
    const std::string                                  var_type_meta          = "var_type";
    const std::string time_meta = "time";
    const std::map<std::string, std::set<std::string>> use_transformed_storage_map = {
        {"ElementBlock", {"connectivity_edge", "connectivity_face"}},
        {"FaceBlock", {"connectivity_edge"}}};
    const std::map<std::string, std::set<std::string>> ignore_fields = {
        {"NodeBlock",
         {"connectivity", "connectivity_raw", "node_connectivity_status", "implicit_ids"}},
        {"ElementBlock", {"implicit_ids"}},
        {"FaceBlock", {"connectivity_raw"}},
        {"EdgeBlock", {"connectivity_raw"}},
        {"CommSet", {"ids"}},
        {"SideSet", {"ids"}},
        {"SideBlock", {"side_ids", "ids", "connectivity", "connectivity_raw"}}};
  };
} // namespace Ioad
#endif

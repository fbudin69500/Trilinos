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

#include <adios2.h>
#include <set>
#include "Ioss_Field.h"           // for Field, etc


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
  std::vector<size_t> steps;
  std::pair<size_t, size_t> node_boundaries;
  size_t component_count = {0};
  Ioss::Field::RoleType role;
  std::string variable_type;
  Ioss::BasicType basic_type;
};

template<>
template_to_basic_type<double>()
{
  return Ioss::BasicType::DOUBLE;
}

template<typename T>
template_to_basic_type<T>()
{
  return Ioss::BasicType::INVALID;
}

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
                               size_t data_size) const override
    {
      return 0;
    }
    int64_t get_field_internal(const Ioss::NodeBlock *nb, const Ioss::Field &field, void *data,
                               size_t data_size) const override
    {
      return 0;
    }
    int64_t get_field_internal(const Ioss::EdgeBlock *eb, const Ioss::Field &field, void *data,
                               size_t data_size) const override
    {
      return 0;
    }
    int64_t get_field_internal(const Ioss::FaceBlock *fb, const Ioss::Field &field, void *data,
                               size_t data_size) const override
    {
      return 0;
    }
    int64_t get_field_internal(const Ioss::ElementBlock *eb, const Ioss::Field &field, void *data,
                               size_t data_size) const override
    {
      return 0;
    }
    int64_t get_field_internal(const Ioss::StructuredBlock *sb, const Ioss::Field &field,
                               void *data, size_t data_size) const override
    {
      return 0;
    }
    int64_t get_field_internal(const Ioss::SideBlock *sb, const Ioss::Field &field, void *data,
                               size_t data_size) const override
    {
      return 0;
    }
    int64_t get_field_internal(const Ioss::NodeSet *ns, const Ioss::Field &field, void *data,
                               size_t data_size) const override
    {
      return 0;
    }
    int64_t get_field_internal(const Ioss::EdgeSet *es, const Ioss::Field &field, void *data,
                               size_t data_size) const override
    {
      return 0;
    }
    int64_t get_field_internal(const Ioss::FaceSet *fs, const Ioss::Field &field, void *data,
                               size_t data_size) const override
    {
      return 0;
    }
    int64_t get_field_internal(const Ioss::ElementSet *es, const Ioss::Field &field, void *data,
                               size_t data_size) const override
    {
      return 0;
    }
    int64_t get_field_internal(const Ioss::SideSet *ss, const Ioss::Field &field, void *data,
                               size_t data_size) const override
    {
      return 0;
    }
    int64_t get_field_internal(const Ioss::CommSet *cs, const Ioss::Field &field, void *data,
                               size_t data_size) const override
    {
      return 0;
    }

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
    void put_field_internal(const std::string &entity_type, const std::string &entity_name,
                            const Ioss::Field &field, void *data, size_t data_size) const;
    void read_meta_data__() override;
    void define_model();
    // void read_region(adios2::IO & bpio);

    int64_t element_global_to_local__(int64_t global) const { return 0; }
    int64_t node_global_to_local__(int64_t global, bool must_exist) const { return 0; }

    template <typename T>
    void put_data(adios2::IO &bpio, const Ioss::Field &field, void *data,
                  const std::string &encoded_name, bool transformed_field) const;
    template <typename T>
    void                       define_model_internal(adios2::IO &bpio, const Ioss::Field &field,
                                                     const std::string &encoded_name);
    template <typename T> void define_entity_internal(const T &entity_blocks, adios2::IO &bpio);

    bool use_transformed_storage(const Ioss::Field &field, const std::string &entity_type, const std::string &field_name) const;

    template <typename T>
    BlockInfoType get_variable_infos(adios2::IO &       bpio,
                                                      const std::string &var_name);
    std::string encode_field_name(const std::string &entity_type, const std::string &entity_name,
                                  const std::string &field_name) const;
    std::tuple<std::string, std::string, std::string>
         decode_field_name(const std::string &encoded_name) const;
    using VariableMapType = std::map<std::string, std::map<std::string, std::map<std::string, std::pair<std::string, std::string>>>>;
    void get_nodeblocks(adios2::IO &reader_io, const VariableMapType &variables);
    void
    // add_attribute_fields(Ioss::GroupingEntity *block, std::vector<std::pair<size_t, size_t>> size,
    //                      std::map<std::string, std::pair<std::string, std::string>> field_names);

    int find_field_in_mapset(const std::string &entity_type, const std::string &field_name, const std::map<std::string, std::set<std::string>> &mapset) const;

    //`ad` needs to be a pointer or a mutable to avoid the following errror:
    // error: passing ‘const adios2::ADIOS’ as ‘this’ argument discards qualifiers
    adios2::ADIOS *                                    ad;
    const std::string                                  schema_version_string = "IOSS_adios_version";
    mutable adios2::Engine                             bp_engine;
    int                                                spatialDimension{0};
    unsigned long                                      rank;
    unsigned long                                      number_proc;
    const std::string                                  name_separator         = "/";
    const std::string                                  attribute_separator    = "::";
    const std::string                                  role_attribute         = "role";
    const std::string                                  var_type_attribute     = "var_type";
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

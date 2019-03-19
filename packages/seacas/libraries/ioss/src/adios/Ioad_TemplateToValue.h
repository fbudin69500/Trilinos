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
#ifndef IOSS_Ioad_TemplateToValue_h
#define IOSS_Ioad_TemplateToValue_h

#include "Ioss_Field.h" // for Field, etc

namespace Ioad
{

  template <typename T> constexpr Ioss::Field::BasicType template_to_basic_type()
  {
    return Ioss::Field::BasicType::INVALID;
  }

  template <> constexpr Ioss::Field::BasicType template_to_basic_type<double>()
  {
    return Ioss::Field::BasicType::DOUBLE;
  }

  template <> constexpr Ioss::Field::BasicType template_to_basic_type<int32_t>()
  {
    return Ioss::Field::BasicType::INT32;
  }

  template <> constexpr Ioss::Field::BasicType template_to_basic_type<int64_t>()
  {
    return Ioss::Field::BasicType::INT64;
  }

  template <> constexpr Ioss::Field::BasicType template_to_basic_type<Complex>()
  {
    return Ioss::Field::BasicType::COMPLEX;
  }

  template <> constexpr Ioss::Field::BasicType template_to_basic_type<std::string>()
  {
    return Ioss::Field::BasicType::STRING;
  }

  template <> constexpr Ioss::Field::BasicType template_to_basic_type<char>()
  {
    return Ioss::Field::BasicType::CHARACTER;
  }

  template <typename T> constexpr char const * get_entity_type(); // Not implemented on purpose
  template <> constexpr char const * get_entity_type<Ioss::SideBlock>() { return "SideBlock"; }

  template <> constexpr char const *get_entity_type<Ioss::SideSet>() { return "SideSet"; }

  template <> constexpr char const *get_entity_type<Ioss::NodeBlock>() { return "NodeBlock"; }

  template <> constexpr char const *get_entity_type<Ioss::EdgeBlock>() { return "EdgeBlock"; }

  template <> constexpr char const *get_entity_type<Ioss::FaceBlock>() { return "FaceBlock"; }

  template <> constexpr char const *get_entity_type<Ioss::ElementBlock>() { return "ElementBlock"; }

  template <> constexpr char const *get_entity_type<Ioss::NodeSet>() { return "NodeSet"; }

  template <> constexpr char const *get_entity_type<Ioss::EdgeSet>() { return "EdgeSet"; }

  template <> constexpr char const *get_entity_type<Ioss::FaceSet>() { return "FaceSet"; }

  template <> constexpr char const *get_entity_type<Ioss::ElementSet>() { return "ElementSet"; }

  template <> constexpr char const *get_entity_type<Ioss::CommSet>() { return "CommSet"; }

} // end of namespace

#endif
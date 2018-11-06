#ifndef IOSS_Ioad_AdiosWrapper_hpp
#define IOSS_Ioad_AdiosWrapper_hpp

#include "adios/AdiosWrapper.h"

namespace Ioad {

  template <typename T>
  void AdiosWrapper::DefineMetaVariable(const std::string &meta_name,
                                        const std::string &variable_name)
  {
    // Meta variables should only be declared and written by process of rank 0 to avoid any
    // undefined behavior.
    if (m_Rank == 0) {
      this->IO::DefineVariable<T>(EncodeMetaVariable(meta_name, variable_name));
    }
  }

  template <typename T>
  void AdiosWrapper::PutMetaVariable(const std::string &meta_name, T value,
                                     const std::string &variable_name)
  {
    // Meta variables should only be declared and written by process of rank 0 to avoid any
    // undefined behavior.
    if (m_Rank == 0) {
      this->Engine::Put<T>(EncodeMetaVariable(meta_name, variable_name), &value,
                           adios2::Mode::Sync); // If not Sync, variables are not saved correctly.
    }
  }

  template <typename T>
  T AdiosWrapper::GetMetaVariable(const std::string &meta_name, const std::string &variable_name)
  {
    T variable;
    this->Engine::Get<T>(EncodeMetaVariable(meta_name, variable_name), variable,
                         adios2::Mode::Sync); // If not Sync, variables are not saved correctly.
    return variable;
  }

} // end of namespace Ioad

#endif
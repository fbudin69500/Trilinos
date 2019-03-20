#ifndef IOSS_Ioad_AdiosWrapper_hpp
#define IOSS_Ioad_AdiosWrapper_hpp

#include "adios/AdiosWrapper.h"

namespace Ioad {

  template <typename T>
  void AdiosWrapper::DefineVariable(const std::string &name, const adios2::Dims &shape,
                                    const adios2::Dims &start, const adios2::Dims &count,
                                    const bool constantDims)
  {
    adios2::Variable<T> var = this->IO::InquireVariable<T>(name);
    if (!var) {
      this->IO::DefineVariable<T>(name, shape, start, count);
    }
  }

  template <typename T>
  void AdiosWrapper::DefineMetaVariable(const std::string &meta_name,
                                        const std::string &variable_name)
  {
    std::string encoded_name = EncodeMetaVariable(meta_name, variable_name);
    adios2::Variable<T> var = this->IO::InquireVariable<T>(encoded_name);
    if (!var) {
      this->IO::DefineVariable<T>(encoded_name);
    }
  }

  template <typename T>
  void AdiosWrapper::PutMetaVariable(const std::string &meta_name, T value,
                                     const std::string &variable_name)
  {
    this->Engine::Put<T>(EncodeMetaVariable(meta_name, variable_name), &value,
                           adios2::Mode::Sync); // If not Sync, variables are not saved correctly.
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
#ifndef IOSS_Ioad_AdiosWrapper_h
#define IOSS_Ioad_AdiosWrapper_h

#include <adios2.h>
#include <string>

namespace Ioad {

  class AdiosWrapper : private adios2::ADIOS, private adios2::IO, private adios2::Engine
  {
  public:
    AdiosWrapper(MPI_Comm communicator, const std::string &filename, bool is_input,
                 unsigned long rank);
    AdiosWrapper(AdiosWrapper &&wrapper);
    ~AdiosWrapper();
    void BeginStep();
    void EndStep();
    template <typename T>
    void DefineMetaVariable(const std::string &meta_name, const std::string &variable_name = "");
    template <typename T>
    void PutMetaVariable(const std::string &meta_name, T value,
                         const std::string &variable_name = "");
    template <typename T>
    T GetMetaVariable(const std::string &meta_name, const std::string &variable_name = "");
    std::pair<std::string, std::string> DecodeMetaName(std::string name) const;
    std::string    EncodeMetaVariable(const std::string &meta_name,
                                      const std::string &variable_name = "") const;

    using adios2::Engine::AllStepsBlocksInfo;
    using adios2::Engine::Get;
    using adios2::Engine::Put;
    using adios2::IO::AvailableVariables;
    using adios2::IO::DefineAttribute;
    using adios2::IO::DefineVariable;
    using adios2::IO::InquireAttribute;
    using adios2::IO::InquireVariable;

  private:
    adios2::IO     IOInit();
    adios2::Engine EngineInit(MPI_Comm communicator, const std::string &filename, bool is_input);


    const std::string m_MetaSeparator{"::"};

    const int m_Rank;
    //  adios2::ADIOS *m_Adios;
    const MPI_Comm m_Communicator;

    // adios2::IO             m_BPIO;
    //  mutable adios2::Engine m_BPEngine;
    bool m_OpenStep;
  };

} // end of namespace Ioad

#include "adios/AdiosWrapper.hpp"

#endif
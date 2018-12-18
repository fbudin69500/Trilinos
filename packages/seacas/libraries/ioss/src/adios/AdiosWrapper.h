#ifndef IOSS_Ioad_AdiosWrapper_h
#define IOSS_Ioad_AdiosWrapper_h

#include <adios2.h>
#include <string>
#include <Ioss_PropertyManager.h>

namespace Ioad {

  class AdiosWrapper : private adios2::ADIOS, private adios2::IO, private adios2::Engine
  {
  public:
    AdiosWrapper(MPI_Comm communicator, const std::string &filename, bool is_input,
                 unsigned long rank, const Ioss::PropertyManager &properties);
    AdiosWrapper(AdiosWrapper &&wrapper);
    ~AdiosWrapper();
    adios2::StepStatus BeginStep();
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

    bool IsStreaming() const {return m_IsStreaming;};

    using adios2::Engine::AllStepsBlocksInfo;
    using adios2::Engine::Get;
    using adios2::Engine::Put;
    using adios2::IO::AvailableVariables;
    using adios2::IO::DefineAttribute;
    using adios2::IO::DefineVariable;
    using adios2::IO::InquireAttribute;
    using adios2::IO::InquireVariable;

  private:
    adios2::IO     IOInit(const Ioss::PropertyManager &properties, bool is_input);
    adios2::Engine EngineInit(const std::string &filename, bool is_input);


    const std::string m_MetaSeparator{"::"};

    const int m_Rank;
    //  adios2::ADIOS *m_Adios;
    const MPI_Comm m_Communicator;

    // adios2::IO             m_BPIO;
    //  mutable adios2::Engine m_BPEngine;
    bool m_OpenStep;
    bool m_IsStreaming;
  };

} // end of namespace Ioad

#include "adios/AdiosWrapper.hpp"

#endif
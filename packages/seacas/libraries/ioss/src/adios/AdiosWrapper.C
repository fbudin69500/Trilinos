#include "adios/AdiosWrapper.h"

namespace Ioad {

  AdiosWrapper::AdiosWrapper(MPI_Comm comm, const std::string &filename, bool is_input,
                             unsigned long rank, const Ioss::PropertyManager &properties)
      : adios2::ADIOS(comm),
        m_Communicator(comm), adios2::IO(IOInit(properties)), adios2::Engine(
                                                        EngineInit(filename, is_input)),
        m_OpenStep(false), m_Rank(rank)
  {
  }

  adios2::IO AdiosWrapper::IOInit(const Ioss::PropertyManager &properties)
  {
    adios2::IO bpio = this->ADIOS::DeclareIO("io");
    std::string engine = "BPFile";
    std::string transport = "File";
    adios2::Params transport_parameters = {{"Library", "POSIX"}};

    if (properties.exists("engine")) {
      engine = properties.get("engine").get_string();
    }
    bpio.SetEngine(engine);
    
    bpio.AddTransport(transport, transport_parameters);
    return bpio;
  }

  adios2::Engine AdiosWrapper::EngineInit(const std::string &filename,
                                          bool is_input)
  {
    adios2::Mode mode = adios2::Mode::Read;
    if (!is_input) {
      mode = adios2::Mode::Write;
    }
    return this->IO::Open(filename, mode);
  }

  AdiosWrapper::~AdiosWrapper()
  {
    EndStep();
    this->Engine::Close();
  }

  void AdiosWrapper::BeginStep()
  {
    if (!m_OpenStep) {
      if (this->Engine::BeginStep() != adios2::StepStatus::OK) {
        std::ostringstream errmsg;
        errmsg << "ERROR: `BeginStep()` did not return OK.\n";
      }
      else {
        // If we are here, `BeginStep()` worked.
        m_OpenStep = true;
      }
    }
    // Either we took called `BeginStep()` successfully or we didn't need to. Either
    // way, there was no issue.
  }

  void AdiosWrapper::EndStep()
  {
    if (m_OpenStep) {
      this->Engine::EndStep();
      m_OpenStep = false;
    }
  }

  std::string AdiosWrapper::EncodeMetaVariable(const std::string &meta_name,
                                               const std::string &variable_name) const
  {
    if (!variable_name.empty()) {
      return variable_name + m_MetaSeparator + meta_name;
    }
    else {
      return meta_name;
    }
  }

  std::pair<std::string, std::string> AdiosWrapper::DecodeMetaName(std::string name) const
  {
    std::size_t pos = 0;
    std::string meta;
    pos = name.rfind(m_MetaSeparator);
    if (pos != std::string::npos && pos != name.size() - 1) {
      meta = name.substr(pos + m_MetaSeparator.size());
      name = name.substr(0, pos);
    }
    return std::make_pair(name, meta);
  }

} // namespace Ioad

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
    std::string library = "POSIX";
    adios2::Params transport_parameters = {{"Library", "POSIX"}};
    // Set engine based on properties
    if (properties.exists("Engine")) {
      engine = properties.get("Engine").get_string();
    }
    bpio.SetEngine(engine);
    // Set transport based on properties
    if (properties.exists("Transport")) {
      transport = properties.get("Transport").get_string();
    }
    // if(transport == "File") // Leave transport parameters to default value.
    if(transport == "WAN") {
      transport_parameters = {{"Library", "ZMQ"}, {"IPAddress", "127.0.0.1"}};
    }
    else if (transport == "InSituMPI") {
      transport_parameters = {};
    }
    else if (transport == "SST") {
      transport_parameters = {{"MarshalMethod", "BP"}};
    }
    // Set transport parameters based on properties
    if (properties.exists("Library")) {
      transport_parameters["Library"] = properties.get("Library").get_string();
    }
    if (properties.exists("IPAddress")) {
      transport_parameters["IPAddress"] = properties.get("IPAddress").get_string();
    }
    if (properties.exists("MarshalMethod")) {
      transport_parameters["MarshalMethod"] = properties.get("MarshalMethod").get_string();
    }
    if (properties.exists("verbose")) {
      transport_parameters["verbose"] = properties.get("verbose").get_string();
    }
    bpio.AddTransport(transport, transport_parameters);
    // TODO: Add support for passing parameters, such as "num_threads".
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

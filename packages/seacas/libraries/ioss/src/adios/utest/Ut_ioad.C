
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include "Ioss_SideBlock.h"
#include "Ioss_NodeBlock.h"
#include "Ioss_SideSet.h"
#include "Ioss_CommSet.h"        // for CommSet
#include "Ioss_EdgeBlock.h"      // for EdgeBlock
#include "Ioss_EdgeSet.h"        // for EdgeSet
#include "Ioss_ElementBlock.h"   // for ElementBlock
#include "Ioss_ElementSet.h"     // for ElementSet
#include "Ioss_FaceBlock.h"      // for FaceBlock
#include "Ioss_FaceSet.h"        // for FaceSet
#include "Ioss_NodeSet.h"         // for NodeSet

#include "Ioss_DatabaseIO.h" // for DatabaseIO

#include "Ioss_IOFactory.h"  // for IOFactory
#include <init/Ionit_Initializer.h>

#include "adios/Ioad_TemplateToValue.h"
#include "adios/Ioad_Helper.h"


#ifdef SEACAS_HAVE_MPI
#include "mpi.h"
#else
#define MPI_COMM_WORLD 1
#endif


int main(int argc, char* argv[])
{
  #ifdef SEACAS_HAVE_MPI
  MPI_Init(&argc, &argv);
  #endif
  const int result = Catch::Session().run(argc, argv);
    #ifdef SEACAS_HAVE_MPI
    MPI_Finalize();
    #endif

  return result;
}

TEST_CASE("Ioad", "[Ioad]")
{

    Ioss::Init::Initializer::initialize_ioss();

    // Ioss::DatabaseIO *db = Ioss::IOFactory::create(
    //       "adios", "noname", Ioss::READ_MODEL, MPI_COMM_WORLD);

}

  template <typename T> const std::string get_entity_type_test()
  {
    // Use "node" as default entity type to enable factory to create object.
    std::unique_ptr<T> entity(Ioad::NewEntity<T>(nullptr, "", "node", 0));
    return entity->type_string();
  }

  // NodeBlock has a different constructor...
  template <> const std::string get_entity_type_test<Ioss::NodeBlock>()
  {
    Ioss::NodeBlock nodeblock(nullptr, "", 0, 1);
    return nodeblock.type_string();
  }

  // SideSet has a different constructor...
  template <> const std::string get_entity_type_test<Ioss::SideSet>()
  {
    Ioss::SideSet sideset(nullptr, "");
    return sideset.type_string();
  }


  template <> const std::string get_entity_type_test<Ioss::SideBlock>()
  {
    // default element arbitrarily set to "hex8": We need an element type
    // to be able to construct this sideblock.
    Ioss::SideBlock sideblock(nullptr, "", "node", "hex8", 0);
    return sideblock.type_string();
  }

TEST_CASE("Ioad_BlockNames", "[Ioad]")
{
  REQUIRE(get_entity_type_test<Ioss::SideBlock>() == Ioad::get_entity_type<Ioss::SideBlock>());
  REQUIRE(get_entity_type_test<Ioss::SideSet>() == Ioad::get_entity_type<Ioss::SideSet>());
  REQUIRE(get_entity_type_test<Ioss::NodeBlock>() == Ioad::get_entity_type<Ioss::NodeBlock>());
  REQUIRE(get_entity_type_test<Ioss::EdgeBlock>() == Ioad::get_entity_type<Ioss::EdgeBlock>());
  REQUIRE(get_entity_type_test<Ioss::FaceBlock>() == Ioad::get_entity_type<Ioss::FaceBlock>());
  REQUIRE(get_entity_type_test<Ioss::ElementBlock>() == Ioad::get_entity_type<Ioss::ElementBlock>());
  REQUIRE(get_entity_type_test<Ioss::NodeSet>() == Ioad::get_entity_type<Ioss::NodeSet>());
  REQUIRE(get_entity_type_test<Ioss::EdgeSet>() == Ioad::get_entity_type<Ioss::EdgeSet>());
  REQUIRE(get_entity_type_test<Ioss::FaceSet>() == Ioad::get_entity_type<Ioss::FaceSet>());
  REQUIRE(get_entity_type_test<Ioss::ElementSet>() == Ioad::get_entity_type<Ioss::ElementSet>());
  REQUIRE(get_entity_type_test<Ioss::CommSet>() == Ioad::get_entity_type<Ioss::CommSet>());



}
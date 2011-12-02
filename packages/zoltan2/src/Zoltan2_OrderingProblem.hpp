
#ifndef _ZOLTAN2_ORDERINGPROBLEM_HPP_
#define _ZOLTAN2_ORDERINGPROBLEM_HPP_

#include <Zoltan2_Problem.hpp>
//#include <Zoltan2_OrderingAlgorithms.hpp> // TODO: Fix include path?
#include "algorithms/order/Zoltan2_OrderingAlgorithms.hpp"
#include <Zoltan2_OrderingSolution.hpp>

#include <Zoltan2_GraphModel.hpp>
#include <string>
#ifdef HAVE_OVIS
#include <ovis.h>
#endif

/*! \file Zoltan2_OrderingProblem.hpp

  This file contains the OrderingProblem class, which derives from 
  the Problem class.
*/

using Teuchos::rcp_dynamic_cast;
using namespace std;

namespace Zoltan2{

////////////////////////////////////////////////////////////////////////
template<typename Adapter>
class OrderingProblem : public Problem<Adapter>
{
public:

  typedef typename Adapter::gid_t gid_t;
  typedef typename Adapter::lid_t lid_t;
  typedef typename Adapter::lno_t lno_t;

  // Destructor
  virtual ~OrderingProblem() {};

  //! Constructor with InputAdapter Interface
  OrderingProblem(Adapter *A, Teuchos::ParameterList *p) 
                      : Problem<Adapter>(A, p) 
  {
    HELLO;
    createOrderingProblem();
  };

  // Other methods
  //   LRIESEN - Do we restate virtual in the concrete class?  I
  //    don't think I've seen this style before.
  virtual void solve();
  // virtual void redistribute();

  OrderingSolution<gid_t, lid_t, lno_t> *getSolution() {
    return solution_.getRawPtr();
  };

private:
  void createOrderingProblem();

  RCP<OrderingSolution<gid_t, lid_t, lno_t> > solution_;

};

////////////////////////////////////////////////////////////////////////
template <typename Adapter>
void OrderingProblem<Adapter>::solve()
{
  HELLO;

  this->solution_ = rcp(new OrderingSolution<gid_t, lid_t, lno_t>);

  // Determine which algorithm to use based on defaults and parameters.
  // For now, assuming RCM.
  // Need some exception handling here, too.

  string method = this->params_->template get<string>("ORDER_METHOD", "RCM");

  // TODO: Ignore case
  if (method.compare("RCM") == 0)
  {
      AlgRCM<Adapter>(this->graphModel_, this->solution_, this->params_,
                      this->comm_);
  }
  else if (method.compare("Minimum_Degree") == 0)
  {
      string pkg = this->params_->template get<string>("ORDER_PACKAGE", "AMD");
      if (pkg.compare("AMD") == 0)
      {
          AlgAMD<Adapter>(this->graphModel_, this->solution_, this->params_,
                          this->comm_);
      }
  }
}

////////////////////////////////////////////////////////////////////////
//template <typename Adapter>
//void OrderingProblem<Adapter>::redistribute()
//{
//  HELLO;
//}

////////////////////////////////////////////////////////////////////////
//! createOrderingProblem 
//  Method with common functionality for creating a OrderingProblem.
//  Individual constructors do appropriate conversions of input, etc.
//  This method does everything that all constructors must do.

template <typename Adapter>
void OrderingProblem<Adapter>::createOrderingProblem()
{
  HELLO;
//  cout << __func__ << " input adapter type " 
//       << this->inputAdapter_->inputAdapterType() << " " 
//       << this->inputAdapter_->inputAdapterName() << endl;

#ifdef HAVE_OVIS
  ovis_enabled(this->comm_->getRank());
#endif

  // Determine which parameters are relevant here.
  // For now, assume parameters similar to Zoltan:
  //   MODEL = graph, hypergraph, geometric, ids
  //   ALGORITHM = rcm, random

  ModelType modelType = GraphModelType;

  // Select Model based on parameters and InputAdapter type
  switch (modelType) {

  case GraphModelType:

    this->graphModel_ = RCP<GraphModel<Adapter> > 
                        (new GraphModel<Adapter>(this->inputAdapter_,
                                                 this->comm_, this->env_,
                                                 false, true));
    break;

  case HypergraphModelType:
  case GeometryModelType:
  case IdModelType:
    cout << __func__ << " Model type " << modelType << " not yet supported." 
         << endl;
    break;

  default:
    cout << __func__ << " Invalid model" << modelType << endl;
    break;
  }
}

}
#endif

#ifdef STATE_ESTIMATOR_NODE_HEADER
template class state_estimation::StateEstimatorNode<robots::UnitreeA1>;
#endif

#ifdef WBIC_NODE_HEADER
template class control::WBICNode<robots::UnitreeA1>;
#endif

#ifdef LOCAL_PLANNER_NODE_HEADER
template class control::LocalPlannerNode<robots::UnitreeA1>;
#endif

#ifdef MOVE_TO_INTERFACE_HEADER
template class interfaces::MoveToInterface<robots::UnitreeA1>;
#endif

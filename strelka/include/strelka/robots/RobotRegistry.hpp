#ifdef STATE_ESTIMATOR_NODE_HEADER
template class state_estimation::StateEstimatorNode<robots::UnitreeA1>;
#undef STATE_ESTIMATOR_NODE_HEADER
#endif

#ifdef WBIC_NODE_HEADER
template class control::WBICNode<robots::UnitreeA1>;
#undef WBIC_NODE_HEADER
#endif

#ifdef LOCAL_PLANNER_NODE_HEADER
template class control::LocalPlannerNode<robots::UnitreeA1>;
#undef LOCAL_PLANNER_NODE_HEADER
#endif

#ifdef MOVE_TO_INTERFACE_HEADER
template class interfaces::MoveToInterface<robots::UnitreeA1>;
#undef MOVE_TO_INTERFACE_HEADER
#endif

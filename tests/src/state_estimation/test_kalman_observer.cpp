#include <A1/constants.hpp>
#include <interfaces/GazeboInterface.hpp>
#include <state_estimation/KalmanFilterObserver.hpp>

#include <iostream>

using strelka::KalmanFilterObserver;

int main() {
  strelka::KalmanFilterObserver::KalmanFilterObserverParams params;
  strelka::KalmanFilterObserver::KalmanFilterObserverInput input;
  strelka::KalmanFilterObserver::KalmanFilterObserverOutput output;

  KalmanFilterObserver observer(params);
  output = observer.update(input);
  print(output.position);
  return 0;
}
#include "drake/examples/pivot/cartesian_controller.h"
#include "cartesian_controller.h"

namespace drake {
    namespace examples {
        namespace pivot {

            template<typename T>
            CartesianController<T>::CartesianController(
                    const multibody::MultibodyPlant <T> &plant,
                    const multibody::ModelInstanceIndex &robot_model_instance_index,
                    std::string body_name, float kp, float kd)
                    : plant_(plant),
                    robot_model_instance_index_(robot_model_instance_index),
                    kp_(kp), kd_(kd) {
                auto plant_context = plant_.CreateDefaultContext();
                q_dim_ = plant_.GetPositions(plant_context.get_value(), robot_model_instance_index_);
            }

        }
    }
}

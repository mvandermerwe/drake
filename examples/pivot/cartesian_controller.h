#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/tree/multibody_element.h"

namespace drake {
    namespace examples {
        namespace pivot {

            template<typename T>
            class CartesianController : public systems::LeafSystem<T> {
                DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CartesianController)

            public:
                CartesianController(
                        const multibody::MultibodyPlant <T> &plant,
                        const multibody::ModelInstanceIndex &robot_model_instance_index,
                        std::string body_name, float kp, float kd
                );

//                void SetMultibodyContext(const systems::Context <T>,
//                                         Context <T> *plant_context) const;
//
//                void CalcOutputForce(const systems::Context <T>, BasicVector <T> *output) const;

            private:
                const multibody::MultibodyPlant <T> &plant_;
                const multibody::ModelInstanceIndex &robot_model_instance_index_;
                float kp_;
                float kd_;
                int q_dim_;
                int v_dim_;
            };

        }
    }
}

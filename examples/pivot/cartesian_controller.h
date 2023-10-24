#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/tree/multibody_element.h"

namespace drake {
    namespace examples {
        namespace pivot {

            template<typename T>
            class CartesianController : public systems::LeafSystem<T> {

            public:
                CartesianController(
                        const multibody::MultibodyPlant<T> *plant,
                        const multibody::ModelInstanceIndex robot_model_instance_index,
                        std::string body_name, float kp, float kd
                );

            private:

                void SetMultibodyContext(const systems::Context<T> &, systems::Context<T> *) const;

                void CalcOutputForce(const systems::Context<T> &, systems::BasicVector<T> *) const;

                const multibody::MultibodyPlant<T> *plant_;
                const multibody::ModelInstanceIndex &robot_model_instance_index_;
                float kp_;
                float kd_;
                int q_dim_;
                int v_dim_;
                systems::InputPort<T> &desired_state_port_;
                systems::InputPort<T> &estimated_state_port_;
                systems::CacheIndex plant_context_cache_index_;
                systems::OutputPort<T> &control_output_port_;
                multibody::Body<T> &ee_body_;
            };

        }
    }
}

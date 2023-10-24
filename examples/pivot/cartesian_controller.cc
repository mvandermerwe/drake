#include "drake/examples/pivot/cartesian_controller.h"
#include "cartesian_controller.h"

namespace drake {
    namespace examples {
        namespace pivot {

            template<typename T>
            CartesianController<T>::CartesianController(
                    const multibody::MultibodyPlant<T> *plant,
                    const multibody::ModelInstanceIndex robot_model_instance_index,
                    std::string body_name, float kp, float kd)
                    : plant_(plant),
                      robot_model_instance_index_(robot_model_instance_index),
                      kp_(kp), kd_(kd) {
                auto plant_context = plant_->CreateDefaultContext();
                q_dim_ = plant_->GetPositions(plant_context.get_value(), robot_model_instance_index_);
                v_dim_ = plant_->GetVelocities(plant_context.get_value(), robot_model_instance_index_);

                // Declare input port for the desired state. 7 for pose, 6 for velocity.
                desired_state_port_ = this->DeclareInputPort("desired_state", systems::kVectorValued, 7 + 6);

                // Declare input port for estimated state of system.
                estimated_state_port_ = this->DeclareInputPort("estimated_state", systems::kVectorValued,
                                                               q_dim_ + v_dim_);

                // Update plant context when new state is provided.
                plant_context_cache_index_ = this->DeclareCacheEntry(
                        "plant_context_cache", *plant_context,
                        &CartesianController::SetMultibodyContext,
                        {this->input_port_ticket(estimated_state_port_)}
                );

                ee_body_ = plant_->GetBodyByName(body_name);

                // Declare output port for the control input.
                control_output_port_ = this->DeclareVectorOutputPort("control", plant_->num_actuators(),
                                                                     &CartesianController::CalcOutputForce,
                                                                     {this->all_input_ports_ticket()});
            }

            template<typename T>
            void CartesianController<T>::SetMultibodyContext(const systems::Context<T> &context,
                                                             systems::Context<T> *plant_context) const {
                const VectorX<T> &x = estimated_state_port_.Eval(context);

                // Set the plant positions and velocities.
                plant_->SetPositionsAndVelocities(plant_context, x);
            }

            template<typename T>
            void CartesianController<T>::CalcOutputForce(const systems::Context<T> &context,
                                                         systems::BasicVector<T> *outputs) const {
                // Get current plant context.
                auto plant_context = this->get_cache_entry(
                        plant_context_cache_index_).template Eval<systems::Context<T>>(context);

                // Get the desired state.
                const VectorX<T> &x_d = desired_state_port_.Eval(context);

                // Get current state of robot end effector.
                auto ee_pose = plant_->EvalBodyPoseInWorld(plant_context, ee_body_);
                auto ee_vel = plant_->EvalBodySpatialVelcotiyInWorld(plant_context, ee_body_);
            }

        }
    }
}

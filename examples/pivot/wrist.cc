//
// Created by markvdm on 10/19/23.
//

#include "drake/common/drake_assert.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant_config.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/examples/pivot/cartesian_controller.h"
#include <iostream>

namespace drake {

    using systems::Context;

    namespace examples {
        namespace pivot {
            namespace {

                int wrist_sim() {
                    // Build simulation setup.
                    systems::DiagramBuilder<double> builder;
                    multibody::MultibodyPlantConfig config;
                    config.contact_model = "hydroelastic";
                    auto [plant, scene_graph] = AddMultibodyPlant(config, &builder);
                    auto parser = multibody::Parser(&plant, &scene_graph);
                    parser.AddModelsFromUrl("package://drake/examples/pivot/assets/wrist.urdf");
                    parser.AddModelsFromUrl("package://drake/examples/pivot/assets/table.urdf");
                    parser.AddModelsFromUrl("package://drake/examples/pivot/assets/pivot.urdf");
                    plant.Finalize();

                    // Build visualizer.
                    auto meshcat = std::make_shared<geometry::Meshcat>();
                    visualization::ApplyVisualizationConfig(
                            visualization::VisualizationConfig(), &builder, nullptr, nullptr, nullptr, meshcat
                    );

                    auto diagram = builder.Build();
                    systems::Simulator<double> simulator(*diagram);

                    Context<double> &context = simulator.get_mutable_context();
                    Context<double> &plant_context =
                            diagram->GetMutableSubsystemContext(plant, &context);

                    VectorX<double> q = (VectorX<double>(plant.num_positions())
                            << 0.15, 0.0, 0.06, 0.0, -1.5708, 0.0, -0.1).finished();
                    plant.SetPositions(&plant_context, q);

                    simulator.Initialize();
                    meshcat->StartRecording();
                    simulator.AdvanceTo(15.0);
                    meshcat->StopRecording();

                    meshcat->PublishRecording();

                    std::cout << "Press enter to continue" << std::endl;
                    std::cin.get();

                    return 0;
                }

            } // namespace
        } // namespace pivot
    } // namespace examples
} //namespace drake

int main(int argc, char *argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    return drake::examples::pivot::wrist_sim();
}
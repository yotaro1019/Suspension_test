// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Demonstration program for a suspension test rig with direct actuation on the
// spindle bodies.
//
// A test rig can be instantiated either from a vehicle JSON specification file
// (by indicating the axle to be used in the rig), or from a test rig JSON
// specification file (with or without a steering mechanism).
//
// Driver inputs for a suspension test rig include left/right rod displacements
// and steering input (the latter being ignored if the tested suspension is not
// attached to a steering mechanism).  These driver inputs can be obtained from
// an interactive driver system (of type ChIrrGuiDriverSTR) or from a data file
// (using a driver system of type ChDataDriverSTR).
//
// See the description of ChSuspensionTestRigPushrod::PlotOutput for details
// on data collected (if output is enabled).
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/utils/ChVehicleIrrApp.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChIrrGuiDriverSTR.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChDataDriverSTR.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "Parameter.h"

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================
// USER SETTINGS
// =============================================================================
std::shared_ptr<Parameter> param;





double post_limit = 0.01;







// Output collection
bool output = true;
double out_step_size = 1e-2;


// =============================================================================
int main(int argc, char* argv[]) {
    param.reset(new Parameter("./params.inp") ); //read params


    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    
    filesystem::path current_dir(filesystem::path().getcwd());
    const std::string current_dir_path = current_dir.str();
    chrono::SetChronoDataPath(CHRONO_DATA_DIR);
    chrono::vehicle::SetDataPath(current_dir_path + "/" + param->Get_inp_dir() + "/" );    
    SetChronoOutputPath("./output");
    std::string out_dir = GetChronoOutputPath();

    // Create tires.
    auto tire_L = ReadTireJSON(vehicle::GetDataFile(param->Get_tire_json()));
    auto tire_R = ReadTireJSON(vehicle::GetDataFile(param->Get_tire_json()));

    // Create the suspension test rig.
    std::unique_ptr<ChSuspensionTestRigPushrod> rig;
    if (param->Get_geom_type() == "geom_veh") {
        // From a vehicle JSON specification file (selecting a particular axle)
        rig = std::unique_ptr<ChSuspensionTestRigPushrod>(
            new ChSuspensionTestRigPushrod(vehicle::GetDataFile( param->Get_veh_json() ), param->Get_test_axle_index(), post_limit, tire_L, tire_R));
    } else if(param->Get_geom_type() == "geom_sus") {
        GetLog() << param->Get_sus_json() << "\n";
        // From a suspension test rig JSON specification file
        rig = std::unique_ptr<ChSuspensionTestRigPushrod>(
            new ChSuspensionTestRigPushrod(vehicle::GetDataFile(param->Get_sus_json()), tire_L, tire_R));
    }
GetLog() << "!!\n";
    rig->SetInitialRideHeight(0.5);

    rig->SetSuspensionVisualizationType(param->Get_parts_viz_type());
    rig->SetWheelVisualizationType(param->Get_wheel_viz_type());
    if (rig->HasSteering()) {
        rig->SetSteeringVisualizationType(param->Get_parts_viz_type());
    }
    rig->SetTireVisualizationType(param->Get_tire_viz_type());

    // Create the vehicle Irrlicht application.
    ChVehicleIrrApp app(rig.get(), L"Suspension Test Rig");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(0.5 * (rig->GetSpindlePos(LEFT) + rig->GetSpindlePos(RIGHT)), 2.0, 0.5);
    app.SetTimestep(param->Get_step_size());

    // Create and attach the driver system.
    std::shared_ptr<ChDriverSTR> driver;
    if (param->Get_driver_mode() == "data") {
        // Driver with inputs from file
        auto data_driver = chrono_types::make_shared<ChDataDriverSTR>(vehicle::GetDataFile( param->Get_driver_data() ));
        driver = data_driver;
    } else if(param->Get_driver_mode() == "gui") {
        // Interactive driver
        auto irr_driver = chrono_types::make_shared<ChIrrGuiDriverSTR>(app);
        irr_driver->SetSteeringDelta(1.0 / 50);
        irr_driver->SetDisplacementDelta(1.0 / 250);
        driver = irr_driver;
    } else {
        GetLog() << "ERROR: Driver mode : " << param->Get_driver_mode() << " is not exist\n";
    }
    rig->SetDriver(driver);

    // Initialize suspension test rig.
    rig->Initialize();

    app.AssetBindAll();
    app.AssetUpdateAll();

    // Set up rig output
    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
        rig->SetSuspensionOutput(true);
        rig->SetSteeringOutput(true);
        rig->SetOutput(ChVehicleOutput::ASCII, out_dir, "output", out_step_size);
        rig->SetPlotOutput(out_step_size);
    }

    // ---------------
    // Simulation loop
    // ---------------

    std::cout << "Rig mass: " << rig->GetMass() << std::endl;

    while (app.GetDevice()->run()) {
        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        app.EndScene();

        // Advance simulation of the rig
        rig->Advance(param->Get_step_size());

        // Update visualization app
        app.Synchronize(tire_L->GetTemplateName(), {rig->GetSteeringInput(), 0, 0});
        app.Advance(param->Get_step_size());

        if (driver->Ended())
            break;
    }

    // Write output file and plot (no-op if SetPlotOutput was not called)
    rig->PlotOutput(out_dir, "output_plot");

    return 0;
}

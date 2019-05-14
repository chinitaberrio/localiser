#include "linear_filter.hpp"

#include <ostream>
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/StdVector>



//LinearFilter::LinearFilter() : LocalisationMethod() {
//}




/* TEST OF THE LINEAR FILTER - APPLY ALSO TO THE GRAPH
 *
 * void TestBayesianTracking()
{
    getRandomGenerator().randomize();

    CDisplayWindowPlots winEKF("Tracking - Extended Kalman Filter", 450, 400);
    CDisplayWindowPlots winPF("Tracking - Particle Filter", 450, 400);

    winEKF.setPos(10, 10);
    winPF.setPos(480, 10);

    winEKF.axis(-2, 20, -10, 10);
    winEKF.axis_equal();
    winPF.axis(-2, 20, -10, 10);
    winPF.axis_equal();

    // Create EKF
    // ----------------------
    CRangeBearing EKF;
    EKF.KF_options.method = kfEKFNaive;

    EKF.KF_options.verbosity_level = mrpt::system::LVL_DEBUG;
    EKF.KF_options.enable_profiler = true;

    // Create PF
    // ----------------------
    CParticleFilter::TParticleFilterOptions PF_options;
    PF_options.adaptiveSampleSize = false;
    PF_options.PF_algorithm = CParticleFilter::pfStandardProposal;
    PF_options.resamplingMethod = CParticleFilter::prSystematic;

    CRangeBearingParticleFilter particles;
    particles.initializeParticles(NUM_PARTICLES);
    CParticleFilter PF;
    PF.m_options = PF_options;

#ifdef SAVE_GT_LOGS
    CFileOutputStream fo_log_ekf("log_GT_vs_EKF.txt");
    fo_log_ekf.printf(
        "%%%% GT_X  GT_Y  EKF_MEAN_X  EKF_MEAN_Y   EKF_STD_X   EKF_STD_Y\n");
#endif

    // Init. simulation:
    // -------------------------
    float x = VEHICLE_INITIAL_X, y = VEHICLE_INITIAL_Y, phi = DEG2RAD(-180),
          v = VEHICLE_INITIAL_V, w = VEHICLE_INITIAL_W;
    float t = 0;

    while (winEKF.isOpen() && winPF.isOpen() && !mrpt::system::os::kbhit())
    {
        // Update vehicle:
        x += v * DELTA_TIME * (cos(phi) - sin(phi));
        y += v * DELTA_TIME * (sin(phi) + cos(phi));
        phi += w * DELTA_TIME;

        v += 1.0f * DELTA_TIME * cos(t);
        w -= 0.1f * DELTA_TIME * sin(t);

        // Simulate noisy observation:
        float realBearing = atan2(y, x);
        float obsBearing =
            realBearing + BEARING_SENSOR_NOISE_STD *
                              getRandomGenerator().drawGaussian1D_normalized();
        printf(
            "Real/Simulated bearing: %.03f / %.03f deg\n", RAD2DEG(realBearing),
            RAD2DEG(obsBearing));

        float realRange = sqrt(square(x) + square(y));
        float obsRange =
            max(0.0, realRange +
                         RANGE_SENSOR_NOISE_STD *
                             getRandomGenerator().drawGaussian1D_normalized());
        printf("Real/Simulated range: %.03f / %.03f \n", realRange, obsRange);

        // Process with EKF:
        EKF.doProcess(DELTA_TIME, obsRange, obsBearing);

        // Process with PF:
        CSensoryFrame SF;
        CObservationBearingRange::Ptr obsRangeBear =
            mrpt::make_aligned_shared<CObservationBearingRange>();
        obsRangeBear->sensedData.resize(1);
        obsRangeBear->sensedData[0].range = obsRange;
        obsRangeBear->sensedData[0].yaw = obsBearing;
        SF.insert(obsRangeBear);  // memory freed by SF.

        EKF.getProfiler().enter("PF:complete_step");
        PF.executeOn(particles, nullptr, &SF);  // Process in the PF
        EKF.getProfiler().leave("PF:complete_step");

        // Show EKF state:
        CRangeBearing::KFVector EKF_xkk;
        CRangeBearing::KFMatrix EKF_pkk;

        EKF.getState(EKF_xkk, EKF_pkk);

        printf(
            "Real: x:%.03f  y=%.03f heading=%.03f v=%.03f w=%.03f\n", x, y, phi,
            v, w);
        cout << "EKF: " << EKF_xkk << endl;

        // Show PF state:
        cout << "Particle filter ESS: " << particles.ESS() << endl;

        // Draw EKF state:
        CRangeBearing::KFMatrix COVXY(2, 2);
        COVXY(0, 0) = EKF_pkk(0, 0);
        COVXY(1, 1) = EKF_pkk(1, 1);
        COVXY(0, 1) = COVXY(1, 0) = EKF_pkk(0, 1);

        winEKF.plotEllipse(
            EKF_xkk[0], EKF_xkk[1], COVXY, 3, "b-2", "ellipse_EKF");

// Save GT vs EKF state:
#ifdef SAVE_GT_LOGS
        // %% GT_X  GT_Y  EKF_MEAN_X  EKF_MEAN_Y   EKF_STD_X   EKF_STD_Y:
        fo_log_ekf.printf(
            "%f %f %f %f %f %f\n", x, y,  // Real (GT)
            EKF_xkk[0], EKF_xkk[1], std::sqrt(EKF_pkk(0, 0)),
            std::sqrt(EKF_pkk(1, 1)));
#endif

        // Draw the velocity vector:
        vector<float> vx(2), vy(2);
        vx[0] = EKF_xkk[0];
        vx[1] = vx[0] + EKF_xkk[2] * 1;
        vy[0] = EKF_xkk[1];
        vy[1] = vy[0] + EKF_xkk[3] * 1;
        winEKF.plot(vx, vy, "g-4", "velocityEKF");

        // Draw PF state:
        {
            size_t i, N = particles.m_particles.size();
            vector<float> parts_x(N), parts_y(N);
            for (i = 0; i < N; i++)
            {
                parts_x[i] = particles.m_particles[i].d->x;
                parts_y[i] = particles.m_particles[i].d->y;
            }

            winPF.plot(parts_x, parts_y, "b.2", "particles");

            // Draw PF velocities:
            float avrg_x, avrg_y, avrg_vx, avrg_vy;

            particles.getMean(avrg_x, avrg_y, avrg_vx, avrg_vy);

            vector<float> vx(2), vy(2);
            vx[0] = avrg_x;
            vx[1] = vx[0] + avrg_vx * 1;
            vy[0] = avrg_y;
            vy[1] = vy[0] + avrg_vy * 1;
            winPF.plot(vx, vy, "g-4", "velocityPF");
        }

        // Draw GT:
        winEKF.plot(vector<float>(1, x), vector<float>(1, y), "k.8", "plot_GT");
        winPF.plot(vector<float>(1, x), vector<float>(1, y), "k.8", "plot_GT");

        // Draw noisy observations:
        vector<float> obs_x(2), obs_y(2);
        obs_x[0] = obs_y[0] = 0;
        obs_x[1] = obsRange * cos(obsBearing);
        obs_y[1] = obsRange * sin(obsBearing);

        winEKF.plot(obs_x, obs_y, "r", "plot_obs_ray");
        winPF.plot(obs_x, obs_y, "r", "plot_obs_ray");

        // Delay:
        std::this_thread::sleep_for(
            std::chrono::milliseconds((int)(DELTA_TIME * 1000)));
        t += DELTA_TIME;
    }
}
*/

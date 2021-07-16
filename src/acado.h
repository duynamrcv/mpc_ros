#include "../acado_mpc_export/acado_common.h"
#include "../acado_mpc_export/acado_auxiliary_functions.h"

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include <iostream>
#include <vector>
#include <stdio.h>

using namespace std;

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   10        /* Number of real-time iterations. */
#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */

#define Ts 0.1 // sampling time
#define Lf 1.0

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

// MPC init functions
vector<vector<double>> init_acado();
void init_weight();
vector<vector<double>> run_mpc_acado(vector<double> states,
                                    vector<double> ref_states,
                                    vector<vector<double>> previous_u);
vector<double> calculate_ref_states(const vector<double> &ref_x,
									const vector<double> &ref_y,
									const vector<double> &ref_q,
                                    const double &reference_vx,
                                    const double &reference_vy,
                                    const double &reference_w);
vector<double> motion_prediction(const vector<double> &cur_states,
                                const vector<vector<double>> &prev_u);
vector<double> update_states(vector<double> state, double vx_cmd, double vy_cmd, double w_cmd);
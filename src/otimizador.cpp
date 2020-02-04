#include "ros/ros.h"
#include "nmpc_robo/otimizador.h"
#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include <stdio.h>

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

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;



bool solver(nmpc_robo::otimizador::Request  &req,
         nmpc_robo::otimizador::Response &res)
{

 // Inicialização das Variáveis
    int i,iter;
    acado_initializeSolver();

    // states
    for (i = 0; i < NX * (N + 1); ++i)
      acadoVariables.x[i] = 0.0;

    // controls
    for (i = 0; i < NU * N; ++i)
      acadoVariables.u[i] = 0.0;

    // reference
    for (i = 0; i < NY * N; ++i)
      acadoVariables.y[i] = 0.0;

    // reference endpoint
    for (i = 0; i < NYN; ++i)
      acadoVariables.yN[i] = 0.0;

  #if ACADO_INITIAL_STATE_FIXED
    // initial values
    for (i = 0; i < NX; ++i)
      acadoVariables.x0[i] = 0.1;
  #endif

int    acado_preparationStep();

// Fim da Inicialização

// Variáveis vindas do nó de controle
acadoVariables.x0[0] = req.b; // Z
acadoVariables.x0[1] = req.c; // ThetaR
acadoVariables.od[0] = req.a; // omega
acadoVariables.od[1] = req.b; // Z
acadoVariables.od[2] = req.c; // ThetaR
//

// Inicio do Solver
for(iter = 0; iter < NUM_STEPS; ++iter)
        {
       acado_feedbackStep();
       acado_printControlVariables();
       acado_printDifferentialVariables();
//       acado_shiftStates(2, 0, 0);
//       acado_shiftControls(0);
       acado_preparationStep();
}
// Enviar a variável de entrada do controle u para ser enviado para o código de controle
res.u = acadoVariables.u[0];

    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "otimizador");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("otimizador", solver);
  ROS_INFO("Otimizador pronto para receber argumentos");
  ros::spin();

  return 0;
}

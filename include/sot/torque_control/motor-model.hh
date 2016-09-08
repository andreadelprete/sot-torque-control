#ifndef MOTORMODEL_H
#define MOTORMODEL_H
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (force_torque_estimator_EXPORTS)
#    define SOTFORCETORQUEESTIMATOR_EXPORT __declspec(dllexport)
#  else
#    define SOTFORCETORQUEESTIMATOR_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTFORCETORQUEESTIMATOR_EXPORT
#endif
namespace dynamicgraph {
  namespace sot {
    namespace torque_control {
        class MotorModel {
        public:
            MotorModel();
            double getCurrent(double torque, double dq, double ddq,
                               double Kt_p,     double Kt_n,
                               double Kf_p=0.0, double Kf_n=0.0,
                               double Kv_p=0.0, double Kv_n=0.0,
                               double Ka_p=0.0, double Ka_n=0.0);
                               
            double getTorque (double current, double dq, double ddq,
                               double Kt_p,     double Kt_n,
                               double Kf_p=0.0, double Kf_n=0.0,
                               double Kv_p=0.0, double Kv_n=0.0,
                               double Ka_p=0.0, double Ka_n=0.0);
        private:
            double smoothSign(double value, double threshold);
        };
    } // namespace torque_control
  } // namespace sot
} // namespace dynamicgraph
#endif // MOTORMODEL_H

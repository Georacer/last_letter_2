 class BeardEngine : public Propulsion
 {
   public:
   double s_prop, c_prop, k_motor, k_omega, k_t_p;
   BeardEngine(Model *);
   void initParam();
   void calcThrust();
   void calcTorque();
 };
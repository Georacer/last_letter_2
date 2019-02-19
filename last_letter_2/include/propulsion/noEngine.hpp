
class NoEngine : public Propulsion
 {
   public:
   NoEngine(Model *);
   void calcThrust();
   void calcTorque();
 };
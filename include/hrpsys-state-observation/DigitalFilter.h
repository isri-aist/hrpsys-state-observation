class DigitalFilter 
{
private:
  bool firsttime;
  double a,b;
  double dt;
  double T;
  double y;
  double dy;
  double ddy;
  double dddy;
  int filter_order;

public:
  void setup(double _T, double _dt){
    // setup first order LPF
    T  = _T;
    dt = _dt;

    if(T != 0.0){
      filter_order = 1;
      a = exp( -dt/T );
      b = 1.0 - a;
    }
    else {
      filter_order = 0;
    }
    
    firsttime = true;
  };

  void setup3(double _T, double _dt){
    // setup 3rd order LPF
    T  = _T;
    dt = _dt;

    if( _T != 0.0 )
      filter_order = 3;
    else
      filter_order = 0;

    firsttime = true;
  };

  void input(double u){
    switch( filter_order ){
    case 0:
      y = u;	// path through
      break;

    case 1:
      // first order LPF
      if( firsttime ){
	firsttime = false;
	y  = u;
      }
      else 
	y  = a * y + b * u;
      break;

    case 3:
      // 3rd order LPF  1/(1+sT)^3
      if( firsttime ){
	firsttime = false;
	y = u;
	dy = ddy = dddy = 0.0;
      }
      else {
	dddy = 1.0/(T*T*T)*(u - y - 3.0*T*dy - 3.0*T*T*ddy);
	y   += dt*dy;
	dy  += dt*ddy;
	ddy += dt*dddy;
      }
      break;
      
    default:
      y = u;	// path through
      break;
    }
  };
  
  double output(){
    return y; 
  };
};

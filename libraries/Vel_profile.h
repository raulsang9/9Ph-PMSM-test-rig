/* 
 * Perfil de velocidad con paso de tiempo variable.
 * Formado por el conjunto de datos [tiempo, pendiente] en Vel_profile.c 
 * siendo el primer valor de pendiente la velocidad inicial 
 * 
 */ 
 
#ifndef VEL_PROFILE 
#define VEL_PROFILE 
 
// Macros 
#define LENVEL 251 

#define SPEEDINIT 62.5581 

#define SPEEDEND 54.9397 

#define TPROF 5.00 

// variable global 
 extern const float vel_prof[LENVEL][2]; 
 
#endif /* VEL_PROFILE */ 
  

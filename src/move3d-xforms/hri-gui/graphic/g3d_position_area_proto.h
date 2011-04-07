#include "../perspective.h"

extern void g3d_draw_rob_pos_area ( void );
extern void g3d_draw_obj_pos_area ( p3d_obj *objPt );
extern void g3d_draw_srchball_pos_area ( psp_searchball *srchballpt );
extern int p3d_is_pos_area_showed ( p3d_rob *r );
extern int p3d_show_rob_pos_area ( p3d_rob *r );
extern int p3d_hide_rob_pos_area ( p3d_rob *r );
extern int p3d_set_robot_pos_area ( p3d_rob *r, double min, double max, double angle );
extern int p3d_set_allhumans_standard_pos_area ( p3d_rob *r );
extern int p3d_is_in_pos_area ( p3d_rob *r, double x, double y, int isrand );
extern int p3d_is_in_obj_pos_area ( p3d_obj *o, double x, double y );
extern double linearDistance ( double x1, double y1, double x2, double y2 );
extern double rad_angleOf ( double x1, double y1, double x2, double y2 );
extern double rad_angleOf_PIMED ( double x1, double y1, double x2, double y2 );
extern double rad2_angleOf ( double x1, double y1, double x2, double y2 );
extern double get_robot_angle_rad ( p3d_rob *r );


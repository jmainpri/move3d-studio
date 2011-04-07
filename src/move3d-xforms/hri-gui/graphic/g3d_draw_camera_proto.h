/*
 *   This file was automatically generated by version 1.7 of cextract.
 *   Manual editing not recommended.
 *
 *   Created: Sun Apr 13 22:53:05 2008
 */
#ifndef __CEXTRACT__

extern void g3d_draw_rob_cone (  p3d_rob *r );
extern int p3d_is_view_field_showed ( p3d_rob *r );
extern void p3d_set_visible_robot_view_field ( p3d_rob *r, int state );
extern void p3d_set_visible_robot_pos_area ( p3d_rob *r, int state );
extern void p3d_set_rob_cam_parameters ( p3d_rob *r, double x, double y, double z, double min, double max, double Vangle, double Hangle, int body, int axe, double pan, double tilt );
extern void p3d_update_rob_cam_parameters ( p3d_rob *r );
extern void set_robot_camera_body ( p3d_rob *r, int body );
extern void p3d_rotVector4_in_axe ( p3d_vector4 point, float theta, int axe, p3d_vector4 result );
extern void gpsp_drawFrustum(float l, float r, float b, float t, float n, float f);
extern void gpsp_draw_robots_fov(G3D_Window  *win);
#endif /* __CEXTRACT__ */
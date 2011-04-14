
extern G3D_Window *g3d_new_win ( const char *name, int w, int h, float size );
extern void g3d_del_win ( G3D_Window *win );
extern int g3d_win_id ( G3D_Window *win );
extern void g3d_refresh_allwin_active ( void );
extern void g3d_event_win ( G3D_Window *g3dwin, int event, int xpos, int ypos, void* data );
extern void g3d_set_dim_light ( void );
extern void g3d_set_default_material();
extern void g3d_set_shade_material();
extern void g3d_draw_win(G3D_Window *win);

extern void (*ext_g3d_draw_allwin_active_backbuffer)();
void g3d_draw_win_back_buffer(G3D_Window *win)
{
	  ext_g3d_draw_allwin_active_backbuffer();
}


extern void g3d_draw_allwin ( void );
extern void g3d_draw_allwin_active ( void );
extern void g3d_draw_allwin_active_back_buffer ( void );
extern void g3d_print_allwin ( void );
extern void g3d_resize_win ( G3D_Window *win, float w, float h, float size );
extern void g3d_resize_allwin_active ( float w, float h, float size );
//extern void g3d_set_win_bgcolor ( G3D_Window *win, float r, float v, float b );
//extern void g3d_set_win_floor_color ( G3D_Window *win, float r, float v, float b );
//extern void g3d_set_win_wall_color ( G3D_Window *win, float r, float v, float b );
//extern void g3d_set_win_camera ( G3D_Window *win, float ox, float oy, float oz, float dist, float az, float el, float up0, float up1, float up2 );
//extern void g3d_set_win_center ( G3D_Window *win, float ox, float oy, float oz );
//extern void g3d_save_win_camera ( G3D_Window *win );
extern void g3d_set_win_fct_mobcam ( G3D_Window *win, pp3d_matrix4 (*fct)(void) );
extern void g3d_set_mobile_camera_activate ( G3D_Window *win, int mode );
extern void g3d_set_win_drawer ( G3D_Window *win, void (*fct)(void) );
extern void g3d_init_allwin_booleans ( void );
extern void g3d_beg_poly ( void );
extern void g3d_end_poly ( void );
extern void g3d_draw_frame ( void );
extern G3D_Window *g3d_get_cur_win ( void );
extern G3D_Window *g3d_get_cmc_win ( void );
extern G3D_Window *g3d_get_win_by_name(char *s);
extern double g3d_get_light_factor(void);
extern void g3d_set_picking(unsigned int enabled);
extern int g3d_export_OpenGL_display(char *filename);
extern void g3d_init_OpenGL();
extern void g3d_set_projection_matrix(g3d_projection_mode mode);
g3d_states& g3d_get_cur_states();
g3d_states& g3d_get_states_by_name(char *s);
extern void g3d_screenshot(char *winname);

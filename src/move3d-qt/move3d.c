#include <Util-pkg.h>
#include <P3d-pkg.h>

#include <move3d-headless.h>

#ifdef P3D_PLANNER
#include <Planner-pkg.h>
#endif
#ifdef P3D_COLLISION_CHECKING
#include <Collision-pkg.h>
#endif

#include <Bio-pkg.h>
#include <UdpClient.h>

#include <locale.h>

// TODO
#undef USE_GLUT
#if defined(USE_GLUT)
#if defined(MACOSX)
#include <glut.h>
#else
#include <GL/freeglut.h>
#endif
#endif

#if defined( QT_GL ) && defined( CXX_PLANNER )
#include <project.hpp>
#include <cppToQt.hpp>
#endif

#include "planner_handler.hpp"

#ifdef LIGHT_PLANNER
#include "../lightPlanner/proto/DlrPlanner.h"
#include "../lightPlanner/proto/DlrParser.h"
#include "../lightPlanner/proto/lightPlannerApi.h"

#if defined( MULTILOCALPATH ) && defined( GRASP_PLANNING )
#include "../lightPlanner/proto/ManipulationTestFunctions.hpp"
#endif
#endif

#include "../graphic/proto/g3d_glut.hpp"

#if defined( CXX_PLANNER ) || defined( MOVE3D_CORE ) 
#include "API/planningAPI.hpp"
#include "planner/cost_space.hpp"
#include <boost/bind.hpp>
#endif
#ifdef BIO_BALL
#include "ball_energy.hpp"
#endif

#include <iostream>
using namespace std;

// External varialbe handled when compiled with move3d studio
bool move3d_studio_load_settings = false;

static int FILTER_TO_BE_SET_ACTIVE = FALSE;
//extern void g3d_create_main_form(void);
//extern void g3d_loop(void);
//extern void p3d_col_set_mode(int);
//extern void kcd_set_user_defined_small_volume(double);
//extern double p3d_get_env_dmax(void);
static void use(void);



int mainMhp(int argc, char ** argv) 
{	
	cout << "Main Mhp" << endl;
	
  //int main(int argc, char ** argv) {
	
	
  // modif Pepijn apropos dmax and tol
  int user_dmax_to_be_set = FALSE;  /* Modif. Pepijn on dmax */
  double user_dmax = 0.0;     /* Modif. Pepijn on dmax */
  int user_obj_tol_to_be_set = FALSE;  /* Modif. Carl on tolerance */
  double user_obj_tol = 0.0;
  // end modif Pepijn
	
  double user_volume, user_size;   /* Modif. Carl on volume */
#ifndef BIO
  const char *file;
#endif
  char file_directory[200];
  char filename[200];
  char scenario[200];
	char dlrReadFile[200];
	char dlrSaveFile[200];
  int i = 0;
  /* carl: */
  int seed_set = FALSE;
  int dir_set = FALSE;
  int file_set = FALSE;
  int scenario_set = FALSE;
  int col_det_set = FALSE;
  int col_mode_to_be_set = p3d_col_mode_kcd; /* Nic p3d_col_mode_v_collide;*/
  int ccntrt_active = TRUE;
  
    // Tests
  int manip_test_run = FALSE;
  int manip_test_id = 0;
  
  // init English C
  if (! setlocale(LC_ALL, "C"))
    fprintf(stderr, "There was an error while setting the locale to \"C\"\n");
	
  int grid_set = FALSE;
  std::string grid_path;
  // modif Brice SALVA
	
#ifdef BIO
  int usrAnswer = FALSE;
  file_name_list* file_list = NULL;
  char name[PSF_MAX_NAME_LENGTH];
  int is_p3d = FALSE;
#endif
  if (! setlocale(LC_ALL, "C")) fprintf(stderr, "There was an error while setting the locale to \"C\"\n");
	
  // End modif Brice SALVA
	
  /* lecture des arguments */
  /* carl: */
  i = 1;
  while (i < argc) {
    if (strcmp(argv[i], "-debug") == 0) {
      basic_alloc_debugon();
      i++;
    } else if (strcmp(argv[i], "-d") == 0) {
      ++i;
      if ((i < argc)) {
        strcpy(file_directory, argv[i]);
        dir_set = TRUE;
        ++i;
      } else {
        use();
        return 0;
      }
    } else if (strcmp(argv[i], "-f") == 0) {
      ++i;
      if ((i < argc)) {
        strcpy(filename, argv[i]);
        file_set = TRUE;
        ++i;
      } else {
        use();
        return 0;
      }
    } else if (strcmp(argv[i], "-sc") == 0) {
      ++i;
      if ((i < argc)) {
        strcpy(scenario, argv[i]);
        scenario_set = TRUE;
        ++i;
      } else {
        use();
        return 0;
      }
    } 
    else if (strcmp(argv[i], "-setgui") == 0) {
      ++i;
      move3d_studio_load_settings = true;
    } 
    else if (strcmp(argv[i], "-nogui") == 0) {
      ++i;
      // Nothing to do, fix this in main
    }
#ifdef P3D_COLLISION_CHECKING
		else if (strcmp(argv[i], "-o") == 0) {
      set_collision_by_object(FALSE);
      ++i;
    } else if (strcmp(argv[i], "-x") == 0) {
      FILTER_TO_BE_SET_ACTIVE = TRUE;
      ++i;
    } else if (strcmp(argv[i], "-nkcdd") == 0) {
      set_return_kcd_distance_estimate(FALSE);
      ++i;
    }
#endif
		else if (strcmp(argv[i], "-s") == 0) {
      ++i;
      if ((i < argc)) {
        p3d_init_random_seed(atoi(argv[i]));
        seed_set = TRUE;
        ++i;
      } else {
        use();
        return 0;
      }
    } else if (strcmp(argv[i], "-v") == 0) {
      ++i;
      if (i < argc) {
				
        user_size = atof(argv[i]);
        user_volume = user_size * user_size * user_size;
#ifdef P3D_COLLISION_CHECKING
        kcd_set_user_defined_small_volume(user_volume);
#endif
        ++i;
      } else {
        use();
        return 0;
      }
    } else if (strcmp(argv[i], "-vol") == 0) {
      ++i;
      if (i < argc) {
#ifdef P3D_COLLISION_CHECKING
        kcd_set_user_defined_small_volume(atof(argv[i]));
#endif
        ++i;
      } else {
        use();
        return 0;
      }
    } else if (strcmp(argv[i], "-tol") == 0) {
      ++i;
      if (i < argc) {
        user_obj_tol = atof(argv[i]) ;
        user_obj_tol_to_be_set = TRUE;
        ++i;
      } else {
        use();
        return 0;
      }
    } else if (strcmp(argv[i], "-dmax") == 0) {
      ++i;
      if (i < argc) {
        user_dmax = atof(argv[i]) ;
        user_dmax_to_be_set = TRUE;
        ++i;
      } else {
        use();
        return 0;
      }
    } else if (strcmp(argv[i], "-stat") == 0) {
      ++i;
#ifdef P3D_PLANNER
      enableStats();
#endif
    } 
#if defined(HRI_COSTSPACE) && defined(QT_GL)
		else if (strcmp(argv[i], "-grid") == 0) {
			++i;
			if ((i < argc)) {
				grid_path = argv[i];
				//printf("Grid is at : %s\n",gridPath.c_str());
				grid_set = TRUE;
				++i;
			} else {
				use();
				return 0;
			}
		}
#endif
	else if (strcmp(argv[i], "-udp") == 0) {
	}
		
#ifdef LIGHT_PLANNER
	else if (strcmp(argv[i], "-no-ccntrt") == 0) {
		printf("Deactivate ccntrt at startup\n");
			ccntrt_active = FALSE;
			++i;
		}
	else if (strcmp(argv[i], "-test") == 0) {
		++i;
		manip_test_run = TRUE;
		if (i < argc) {
			manip_test_id = atoi(argv[i]) ;
			++i;
		} else {
			manip_test_run = FALSE;
			use();
			return 0;
		}
	}
#endif
		
	else if (strcmp(argv[i], "-udp") == 0) {
		std::string serverIp(argv[i+1]);
		int port = 0;
		sscanf(argv[i+2], "%d", &port);
		globalUdpClient = new UdpClient(serverIp, port);
		i += 3;
	}
	else if (strcmp(argv[i], "-dlr") == 0) {
			strcpy(dlrReadFile, argv[i + 1]);
			strcpy(dlrSaveFile, argv[i + 2]);
      i += 3;
    }	else if (strcmp(argv[i], "-c") == 0) {
      ++i;
			cout << "Coll mod " << argv[i] << endl;
      if (strcmp(argv[i], "vcollide") == 0) {
        col_mode_to_be_set = p3d_col_mode_v_collide;
        col_det_set = TRUE;
        ++i;
      }
#ifdef P3D_COLLISION_CHECKING
			else if (strcmp(argv[i], "kcd") == 0) {
        col_mode_to_be_set = p3d_col_mode_kcd;
        set_DO_KCD_GJK(TRUE);
        col_det_set = TRUE;
        ++i;
      }
#ifdef PQP
			else if (strcmp(argv[i], "pqp") == 0) {
				printf("Colmod pqp");
				col_mode_to_be_set= p3d_col_mode_pqp;
				col_det_set = TRUE;
				++i;
				cout << "Coll mod PQP " << endl;
      }
#endif
#endif
			else if (strcmp(argv[i], "bio") == 0) {
        col_mode_to_be_set = p3d_col_mode_bio;
        col_det_set = TRUE;
        ++i;
      } else if (strcmp(argv[i], "kng") == 0) {
        col_mode_to_be_set = p3d_col_mode_kcd;
#ifdef P3D_COLLISION_CHECKING
        set_DO_KCD_GJK(FALSE);
#endif
        col_det_set = TRUE;
        ++i;
      } else if (strcmp(argv[i], "gjk") == 0) {
        col_mode_to_be_set = p3d_col_mode_gjk;
        col_det_set = TRUE;
        ++i;
      } else if (strcmp(argv[i], "none") == 0) {
        col_mode_to_be_set = p3d_col_mode_none;
        col_det_set = TRUE;
        ++i;
      } else {
        use();
        return 0;
      }
    } else {
      use();
      return 0;
    }
  }
	
  if (!dir_set)
    strcpy(file_directory, "../../demo");
	
  if (!seed_set)
    p3d_init_random();
	
  if (!col_det_set){
    // modif Juan
		
		//check that the HOME_MOVE3D environment variable exists:
		if(getenv("HOME_MOVE3D")==NULL) {
			printf("%s: %d: main(): The environment variable \"HOME_MOVE3D\" is not defined. This might cause some problems or crashes (e.g. with video capture).\n", __FILE__,__LINE__);
		}
		
		
#ifdef GRASP_PLANNING
		col_mode_to_be_set= p3d_col_mode_pqp;
#ifndef PQP
		printf("%s: %d: main(): GRASP_PLANNING must be compiled with PQP.\n", __FILE__,__LINE__);
		printf("Program must quit.\n");
		return 1;
#endif
#else
    col_mode_to_be_set = p3d_col_mode_kcd;
#endif
  }
#ifdef P3D_COLLISION_CHECKING
  if (col_mode_to_be_set != p3d_col_mode_v_collide)
    set_collision_by_object(FALSE);
  /* : carl */
  if (col_mode_to_be_set == p3d_col_mode_v_collide) {
    p3d_filter_switch_filter_mechanism(FILTER_TO_BE_SET_ACTIVE);
  }
  /* begin added KCD FILTER */
  else if (col_mode_to_be_set == p3d_col_mode_kcd) {
    p3d_filter_switch_filter_mechanism(FILTER_TO_BE_SET_ACTIVE);
  }
#endif
  /*  end  added KCD FILTER */
	
	
  /* lecture du fichier environnement */
  p3d_set_directory(file_directory);

  // init English C
  if (! setlocale(LC_ALL, "C"))
    fprintf(stderr, "There was an error while setting the locale to \"C\"\n");
	
	
  while (!p3d_get_desc_number(P3D_ENV)) {
		
#ifdef BIO
    if (file_set == TRUE) {
			//       if (!filename) {
			//         exit(0);
			//       }
			
#ifndef PQP
      p3d_col_set_mode(p3d_col_mode_none);
#else
      p3d_col_set_mode(col_mode_to_be_set);
#endif
      p3d_BB_set_mode_close();
      
      if (!p3d_read_desc(filename)) 
      {
        file_set = FALSE;
      }
    }
    if (file_set == FALSE) {

      printf("Error : give a p3d filename as argument, or use the XFORMS module.\n");
      exit(0);
    }
#else
		
    if (file_set == TRUE) {
      file = filename;
    } else {

    }
    if (!file) {
      exit(0);
    }
	  
#ifdef P3D_COLLISION_CHECKING
    p3d_col_set_mode(p3d_col_mode_none);
    p3d_BB_set_mode_close();
#endif
		
		printf("\n");
		printf("  ----------------------------\n");
		printf("  -- p3d file parsing start --\n");
		printf("  ----------------------------\n");
		printf("\n");
		
		p3d_read_desc((char *) file);
		
#endif
    if (!p3d_get_desc_number(P3D_ENV)) {
			printf("loading done...\n");

    }
  }
	
	printf("\n");
	printf("  ----------------------------\n");
	printf("  -- p3d file parsing done  --\n");
	printf("  ----------------------------\n");
	printf("\n");
	
  MY_ALLOC_INFO("After p3d_read_desc");
	
  printf("Nb poly : %d\n", p3d_poly_get_nb());
	
  /* for start-up with currently chosen collision detector: */
  /* MY_ALLOC_INFO("Before initialization of a collision detector"); */
#ifdef P3D_COLLISION_CHECKING
  p3d_col_set_mode(col_mode_to_be_set);
  p3d_col_start(col_mode_to_be_set);
#endif
	

#if defined( MOVE3D_CORE ) 
	if (ENV.getBool(Env::isCostSpace))
	{
		GlobalCostSpace::initialize();
	}
#endif
			
  /* modif Pepijn april 2001
	 * this changes have to be made after the initialistion of the collision checker
	 * because in p3d_col_start KCD migth be initialised, and during this initialisation
	 * there is automaticly calculated a dmax
	 * So if the user wants to set his own dmax this most be done after this initialisation
	 * INTERNAL NOTE: in this case the users are the developpers of MOVE3D, normally the clients
	 * who purchase Move3d don't know that this option exists
	 */
  if (user_dmax_to_be_set) {
    if (user_dmax < EPS4) {
      printf("WARNING: User chose dmax too small --> new value set to 0.0001 (EPS4)\n");
      user_dmax = EPS4;
    }
    p3d_set_env_dmax(user_dmax);
  }
  if (user_obj_tol_to_be_set) {
    if (user_obj_tol < 0.0) {
      printf("WARNING: Negative tolerance, tolerance is set to 0.0\n");
      user_obj_tol = 0.0;
    }
    p3d_set_env_object_tolerance(user_obj_tol);
  }
	
  printf("Env dmax = %f\n",p3d_get_env_dmax());
  printf("Env Object tol = %f\n",p3d_get_env_object_tolerance());
  /* always set tolerance even if the user didn't specify any options
   * it's possible that Kcd has calculated automaticly a dmax
   */
  /* Normally  p3d_col_set_tolerance(); is called when initialising
   * the sliders, in case the sliders are not used we have to use
   * p3d_col_set_tolerance()
   */
	
  //printf("MAX_DDLS  %d\n", MAX_DDLS);
	
  // modif Juan
#ifdef BIO
  if (col_mode_to_be_set == p3d_col_mode_bio) {
    bio_set_num_subrobot_AA();
    bio_set_num_subrobot_ligand();
    bio_set_bio_jnt_types();
    bio_set_bio_jnt_AAnumbers();
    bio_set_list_AA_first_jnt();
    bio_set_AAtotal_number();
    bio_set_nb_flexible_sc();
    bio_set_list_firstjnts_flexible_sc();
    if (XYZ_ROBOT->num_subrobot_ligand != -1)
      bio_set_nb_dof_ligand();
#ifdef BIO_BALL
    new BallEnergy(XYZ_ENV);
    if(global_costSpace)
    {
      global_costSpace->addCost("BALLmmff94",
																boost::bind(&BallEnergy::computeEnergy, XYZ_ENV->energyComputer, _1));
      global_costSpace->setCost("BALLmmff94");
    }
#endif
  }
#endif
  // fmodif Juan
	
#if defined(LIGHT_PLANNER) && defined(FK_CNTRT)
	for(int i=0; i<XYZ_ENV->nr; i++)
	{  p3d_create_FK_cntrts(XYZ_ENV->robot[i]);  }
#endif
	
#ifdef P3D_CONSTRAINTS
  // Modif Mokhtar Initialisation For Multisolutions constraints
  p3d_init_iksol(XYZ_ROBOT->cntrt_manager);
#endif
	
  /* creation du FORM main */

  /*
   * needs to be run after main form has been created
   */
  if (scenario_set == TRUE) 
  {
		p3d_rw_scenario_init_name();
		p3d_read_scenario(scenario);
  }
	
  //Set the robots to initial Pos if defined
  for(i = 0; i < XYZ_ENV->nr; i++){
    if(!p3d_isNullConfig(XYZ_ENV->robot[i], XYZ_ENV->robot[i]->ROBOT_POS)){
      p3d_set_and_update_this_robot_conf(XYZ_ENV->robot[i], XYZ_ENV->robot[i]->ROBOT_POS);
    }
  }
	//Exection Of Dlr Planner
	//	do{
	//		DlrPlanner* planner = new DlrPlanner(dlrSaveFile);
	//		DlrParser parser(dlrReadFile, planner);
	//		if(parser.parse()){
	//			planner->process();
	//		}else{
	//			sleep(2);
	//		}
	//		free(planner);
	//	}while(1);
  /* go into loop */
	
#if defined(LIGHT_PLANNER) && defined(PQP)
	if(ENV.getBool(Env::startWithFKCntrt))
	{
		p3d_rob* MyRobot = p3d_get_robot_by_name_containing ( ( char* ) "ROBOT" );
		deactivateCcCntrts(MyRobot,-1);
	}
#endif
	
#if defined(HRI_COSTSPACE) && defined(QT_GL)
	if(grid_set)
	{
		std::cout << "Reading HRICS Grid " << grid_path << std::endl;
		qt_load_HRICS_Grid(grid_path);
	}
#endif
	
	//  double c, color[4];
	//  srand(time(NULL));
	//  c= rand()/((double)RAND_MAX+1);
	//  g3d_rgb_from_hue(c, color);
	//  g3d_set_win_floor_color(g3d_get_cur_win(), color[0], color[1], color[2]);

#if defined( LIGHT_PLANNER ) && !defined( QT_GL )
	if (manip_test_run) 
	{
		printf("Test functions : ManipulationTestFunctions\n");
 		new qtG3DWindow();
    
#if defined( MOVE3D_CORE )
    // Creates the wrapper to the project 
    // Be carefull to initialize in the right thread
    global_Project = new Project(new Scene(XYZ_ENV));
#endif
		
		ManipulationTestFunctions tests;
		
		if(!tests.runTest(manip_test_id))
		{
			std::cout << "ManipulationTestFunctions::Fail" << std::endl;
		}
	}
#endif
  return 0;
}

/* fonction de rappel des formats des arguments */
static void use(void) {
	
  printf("main : ERROR : wrong arguments !\n move3d -d data_directory -f file -sc scenario -s random_seed -c collision_detector -tol tolerance -dmax dist [-v size | -vol volume] -o -x -nkcdd\n");
  printf("collision_detector may be: icollide, vcollide, solid, kcd, kng, none\n");
  printf("(kng == kcd without gjk)\n");
  printf("-o: consider polyhedrons as objects (only for vcollide)\n");
  printf("-x: do filtering by robot body (default: don't filter)\n");
  printf("-v: (only with kcd) minimal relevant size (in 1 dimension) \n");
  printf("-dmax: for Vcollide & nkcdd  => maximum penetration distance \n");
  printf("           KCD               => artificial enlargement of the obstacles \n");
  printf("-tol: (only with kcd) object tolerance (enlarge objects for the collision checker) \n");
  printf("-vol: (only with kcd) minimal relevant size (volume) \n");
  printf("(internal note, kcd: volume of a box around a facet == (surface of box)^(3/2) )\n");
  printf("-nkcdd: (only with -c kcd) return only boolean reply, don't use distance \n");
  printf("-no-ccntrt: deactivate closed chain constraint at startup \n");
}


#include "extIncludes.h"
#ifndef __H_CLASSES_BASE
#define __H_CLASSES_BASE


#define __GRID_CIRCLE_SEGMENTS 100

//typedef boost::geometry::model::d2::point_xy<double> ofVec2d;

class random_base {
	public:
		/*
		 * boost/random/uniform_smallint.hpp
		 */
		random_base(
				boost::random::mt19937* iGen,
				std::string iType,
				long long iA,
				long long iB);
		/*
		 * boost/random/uniform_real_distribution.hpp
		 * boost/random/normal_distribution.hpp
		 * boost/random/lognormal_distribution.hpp
		 */
		random_base(
				boost::random::mt19937* iGen,
				std::string iType,
				double iA,
				double iB);
		/*
		 * boost/random/bernoulli_distribution.hpp
		 * boost/random/exponential_distribution.hpp
		 */
		random_base(
				boost::random::mt19937* iGen,
				std::string iType,
				double iA);
		/*
		 * boost/random/uniform_01.hpp
		 */
		random_base(
				boost::random::mt19937* iGen,
				std::string iType);
		template<class T> T draw ();
	protected:
		unsigned type;
		boost::random::mt19937* gen;
		boost::variant<
			boost::random::uniform_smallint<>*,
			boost::random::uniform_01<>*,
			boost::random::uniform_real_distribution<>*,
			boost::random::bernoulli_distribution<>*,
			boost::random::normal_distribution<>*,
			boost::random::lognormal_distribution<>*,
			boost::random::exponential_distribution<>*
		> dist;
};

class random_container {
	public:
		random_container();
		~random_container();
		unsigned long long& get_seed();
		void set_seed();
		void set_seed(unsigned long long iSeed);
		template<typename... A > random_base* register_random(
				std::string iType,
				A... args
		);
		void unregister_random(random_base* iDist);
	protected:
		unsigned long long seed;
		boost::random::mt19937 gen;
		std::set<random_base*> distributions;
};

/*
 *  The Grid is supposed to ease up workload for finding near neighbors and to increase overall performance
 *  The grid contains cells and each grid cell stores pointers to all components which are located in the gird cell
 */

class components_base;
class grid_cell {
	public:
		grid_cell();
		virtual ~grid_cell();
		std::set<components_base*>& get_components();
		void obtain_intersecting(components_base* iComponentA,components_base* iComponentB);
		void update_intersecting();
		void remove_component(components_base* iComponent);
		void add_component(components_base* iComponent);
	protected:
		std::pair<components_base*,ofVec2d> obtain_intersectingCircleLine(components_base* iRef, components_base* iCom);
		std::pair<components_base*,std::pair<ofVec2d,ofVec2d>> obtain_intersectingLineLine(components_base* iRef, components_base* iCom);
		std::set<components_base*> components;
		typedef boost::geometry::model::point<
				double,
				2,
				boost::geometry::cs::cartesian
		> point_t;
		boost::geometry::model::box<point_t> boxA;
		boost::geometry::model::box<point_t> boxB;
		boost::geometry::model::ring<point_t> ringA;
		boost::geometry::model::ring<point_t> ringB;
		boost::geometry::model::linestring<point_t> lineA;
		boost::geometry::model::linestring<point_t> lineB;
};

class grid_base {
	public:
		//grid_base();
		grid_base(unsigned long long iResolution, double iSideLength);
		virtual ~grid_base();
		void register_component(components_base* iComponent);
		void unregister_component(components_base* iComponent);
		void update_component(components_base* iComponent);
		void update_components();
	protected:
		bool lineSegmentIntersection();
		std::vector<std::vector<grid_cell>> cells;
		std::set<components_base*> components;
		double sideLength;
		unsigned long long resolution;
};

/*
 *  Those are all accessible visual elements which can be used to represent the components.
 *  The name might be misleading as the visual element also determines the physical form of the component.
 */

class visual_base {
	public:
		//visual_base();
		visual_base(unsigned iType, components_base* iComponent);
		virtual ~visual_base();

		virtual components_base& get_associatedComponent();
		virtual ofFloatColor& get_color();
		virtual ofFloatColor& get_fillColor();
		virtual unsigned& get_type();
		virtual std::vector<ofVec2d>& get_positions();
		virtual std::vector<double>& get_parameters();

		virtual void set_associatedComponent(components_base* iComponent);
		virtual void set_color(double iRed,double iGreen, double iBlue, double iAlpha);
		virtual void set_color(double iRed,double iGreen, double iBlue);
		virtual void set_fillColor(double iRed,double iGreen, double iBlue, double iAlpha);
		virtual void set_fillColor(double iRed,double iGreen, double iBlue);
	protected:
		components_base* associatedComponent;
		ofFloatColor color;
		ofFloatColor fillColor;
		unsigned type;
};

class visual_line:public visual_base {
	public:
		visual_line(components_base* iComponent);
		virtual ~visual_line();
	protected:
};
class visual_ellipse:public visual_base {
	public:
		visual_ellipse(components_base* iComponent);
		virtual ~visual_ellipse();
	protected:
};
class visual_rectangle:public visual_base {
	public:
		visual_rectangle(components_base* iComponent);
		virtual ~visual_rectangle();
	protected:
};
class visual_triangle:public visual_base {
	public:
		visual_triangle(components_base* iComponent);
		virtual ~visual_triangle();
	protected:
};

/*
 * Base function for all physical components.
 * It is the base call for cells and substrates
 */

class components_base {
	public:
		components_base(grid_base* iGrid);

		virtual ~components_base();

		virtual bool& get_canMove();
		virtual bool& get_canColide();

		virtual std::vector<ofVec2d>& get_positions();
		virtual std::vector<double>& get_parameters();
		virtual unsigned long long& get_timeStamp();
		virtual std::set<std::type_info>& get_ignoreIntersect();
		virtual std::set<components_base*>& get_intersectorsChecked();
		virtual std::vector<grid_cell*>& get_gridCells();
		virtual visual_base* get_visualObj();

		virtual void set_canMove(bool iCanMove);
		virtual void set_canColide(bool iCanColide);
		virtual void set_gridCells(std::vector<grid_cell*> iGridCells);

		virtual void obtain_visualObjs(std::vector<visual_base*>& iVisualObjs);
		virtual void clear_intersectors();
		virtual void add_intersector(components_base* iIntersector, ofVec2d iIntersectorVec);
		virtual void add_ignoreIntersect(std::type_info iIgnore);
		virtual void make_timeStep(double iTime, unsigned long long iTimeStamp);
	protected:
		virtual void update_timeStamp();
		bool canMove; // is it a fixed object
		bool canColide; // can this object collide aka does it have physics?
		std::vector<ofVec2d> positions; // position of object
		std::vector<double> parameters; // additional parameters of the object
		unsigned long long timeStamp; // timestamp is relevant for update features
		visual_base* associatedVisualObj; // assignes a visual object
		std::set<std::type_info> ignoreIntersect; // ignore class types for collision
		std::vector<components_base*> intersectors; // list of objects which intersect with
		std::vector<ofVec2d> intersectorsVectors; // list of collision vectors of intersectors
		std::set<components_base*> intersectorsChecked;
		std::vector<grid_cell*> gridCells; // gridcells in which object lies
		grid_base* grid;
};

class cell_base:public components_base {
	public:
		cell_base(grid_base* iGrid);
		virtual ~cell_base();
	protected:
};

class matrix_base:public components_base {
	public:
		matrix_base(grid_base* iGrid);
		virtual ~matrix_base();
	protected:
};

/*
 * Fillaments
 */

class fillament_base;
class crosslinker_base : public cell_base {
	public:
		crosslinker_base(grid_base* iGrid);
		virtual ~crosslinker_base();

		virtual std::set<fillament_base*>& get_connectedFillaments();
		virtual ofVec2d& get_force(unsigned long long iTimeStamp);

		virtual void add_connectedFillament(fillament_base* iFillament);
		virtual void remove_connectedFillament(fillament_base* iFillament);
	protected:
		std::set<fillament_base*> connectedFillaments;
		ofVec2d force;
};
class crosslinker_static : public crosslinker_base {
	public:
		crosslinker_static(grid_base* iGrid);
		virtual ~crosslinker_static();
};
class crosslinker_friction : public crosslinker_base {
	public:
		crosslinker_friction(grid_base* iGrid);
		virtual ~crosslinker_friction();
};

class fillament_base : public cell_base {
	public:
		fillament_base(grid_base* iGrid);
		virtual ~fillament_base();
		virtual void set_positions(double iX1, double iY1, double iX2, double iY2);
		virtual void add_connectedCrosslinker(crosslinker_base* iCrosslinker);
		virtual void remove_connectedCrosslinker(crosslinker_base* iCrosslinker);
		virtual void make_timeStep(double& iTime);
	protected:
		std::set<crosslinker_base*> connectedCrosslinkers;
};
class actin : public fillament_base {
	public:
		actin(
				grid_base* iGrid,
				ofVec2d& iStart,
				ofVec2d& iTmVelocity,
				double iMaxLength,
				double iLifeTime,
				double iStallingForce);
		virtual ~actin();
		virtual ofVec2d& get_force(unsigned long long iTimeStamp);
		virtual void make_timeStep(double iTime, unsigned long long iTimeStamp);
	protected:
		ofVec2d tmVelocity; // treadmilling velocity
		ofVec2d force; // current force vector in actin element
		double maxLength; // maximum length
		double lifeTime; // dies after lifetime
		double stallingForce;
		actin* tail;
};

class volume_base : public cell_base {
	public:
		volume_base(grid_base* iGrid);
		virtual ~volume_base();
	protected:
};

/*membrane*/
class membrane_part : public cell_base {
	public:
		// 2D membrane part
		membrane_part(
				grid_base* iGrid,
				double iX1,double iY1,
				double iX2,double iY2
		);
		virtual ~membrane_part();
		virtual void obtain_visualObjs(std::vector<visual_base*>& iVisualObjs);
		virtual void set_neighbours(membrane_part& iPartA,membrane_part& iPartB);
		virtual void make_timeStep(double iTime, unsigned long long iTimeStamp);
	protected:
		std::vector<membrane_part*> neighbours;
};
class membrane_base : public cell_base {
	public:
		membrane_base(
				grid_base* iGrid,
				double iX, double iY,
				double iRadius,
				unsigned long long iResolution
		);
		virtual ~membrane_base();
		virtual double get_volume();
		virtual void obtain_visualObjs(std::vector<visual_base*>& oVisualComponents);
		virtual double calc_currentVolume();
		virtual void make_timeStep(double iTime, unsigned long long iTimeStamp);
	protected:
		bool updatedVolume;
		double initialVolume;
		double currentVolume;
		const bool canColide = true;
		std::vector<membrane_part*> parts;
};

class cell : public cell_base {
	public:
		cell(
				grid_base* iGrid,
				double iX,
				double iY,
				unsigned long long iResolution
		);
		virtual ~cell();
		virtual void obtain_visualObjs(std::vector<visual_base*>& iVisualObjs);
		virtual void make_timeStep(double iTime, unsigned long long iTimeStamp);
	protected:
		double maxFillamentLength;
		std::set<membrane_base*> membrane;
		std::set<fillament_base*> fillaments;
		std::set<volume_base*> volumes;
};

class fac:public matrix_base {
	public:
		fac(
				grid_base* iGrid,
				double iRadius,
				double iX,
				double iY
		);
		virtual ~fac();
		virtual void obtain_visualObjs(std::vector<visual_base*>& iVisualObjs);
		virtual void set_radius(double iRadius);
		virtual void set_position(double iX, double iY);
	protected:
};
class simple_surface;
class surface_border : public matrix_base {
	public:
		surface_border(grid_base* iGrid, simple_surface* iSurface, ofVec2d iStart, ofVec2d iEnd);
		virtual ~surface_border();
		virtual void obtain_visualObjs(std::vector<visual_base*>& iVisualObjs);
	protected:
		simple_surface* surface;
};

class simple_surface : public matrix_base {
	public:
		simple_surface(grid_base* iGrid,double iSideLength);
		virtual ~simple_surface();

		virtual void obtain_visualObjs(std::vector<visual_base*>& iVisualObjs);
		virtual void create_facs(unsigned iType, unsigned long long iCount, double iRadius);
	protected:
		double sideLength;
		std::vector<surface_border*> borders;
		std::vector<fac*> facs;
};

#endif

#pragma once

#include "visitor.h"
#include "enums.h"
#include "memory.h"
#include "Geometry.h"

#include <utility>
#include <list>
#include <vector>
#include <map>

#include <mutex>
#include <future>

class GeometryEvaluator : public Visitor
{
public:
     GeometryEvaluator(const class Tree &tree);
     virtual ~GeometryEvaluator() {}

     shared_ptr<const Geometry> evaluateGeometry(const AbstractNode &node, bool allownef);

     virtual Response visit(State &state, const AbstractNode &node);
     virtual Response visit(State &state, const AbstractIntersectionNode &node);
     virtual Response visit(State &state, const AbstractPolyNode &node);
     virtual Response visit(State &state, const LinearExtrudeNode &node);
     virtual Response visit(State &state, const RotateExtrudeNode &node);
     virtual Response visit(State &state, const LeafNode &node);
     virtual Response visit(State &state, const TransformNode &node);
     virtual Response visit(State &state, const CsgNode &node);
     virtual Response visit(State &state, const CgaladvNode &node);
     virtual Response visit(State &state, const ProjectionNode &node);
     virtual Response visit(State &state, const RenderNode &node);
     virtual Response visit(State &state, const TextNode &node);
     virtual Response visit(State &state, const OffsetNode &node);

     shared_ptr<const class Geometry> do_render_node(const RenderNode &node);
     shared_ptr<const Geometry> do_linear_extrude(const LinearExtrudeNode &node);
     shared_ptr<const Geometry> do_csg_node(const CsgNode &node);
     shared_ptr<const Geometry> do_transform_node(const TransformNode &node);
     shared_ptr<const Geometry> do_rotate_extrude(const RotateExtrudeNode &node);
     shared_ptr<const class Geometry> do_projection_node(const ProjectionNode &node);
     shared_ptr<const Geometry> do_cgaladv_node(const CgaladvNode &node);
     shared_ptr<const class Geometry> do_abstract_intersection_node(const AbstractIntersectionNode &node);

     const Tree &getTree() const { return this->tree; }

private:
     class ResultObject {
     public:
          ResultObject() : is_const(true) {}
          ResultObject(const Geometry *g) : is_const(true), const_pointer(g) {}
          ResultObject(shared_ptr<const Geometry> &g) : is_const(true), const_pointer(g) {}
          ResultObject(Geometry *g) : is_const(false), pointer(g) {}
          ResultObject(shared_ptr<Geometry> &g) : is_const(false), pointer(g) {}
          bool isConst() const { return is_const; }
          shared_ptr<Geometry> ptr() { assert(!is_const); return pointer; }
          shared_ptr<const Geometry> constptr() const {
               return is_const ? const_pointer : static_pointer_cast<const Geometry>(pointer);
          }
     private:
          bool is_const;
          shared_ptr<Geometry> pointer;
          shared_ptr<const Geometry> const_pointer;
     };


     typedef std::pair<const class AbstractNode *, std::shared_future<shared_ptr<const Geometry> > > ChildItem;
     typedef std::list<ChildItem> ChildList;

     void smartCacheInsert(const AbstractNode &node, const shared_ptr<const Geometry> &geom);
     shared_ptr<const Geometry> smartCacheGet(const AbstractNode &node, bool preferNef = false);
     bool isSmartCached(const AbstractNode &node);
     std::vector<const class Polygon2d *> collectChildren2D(const AbstractNode &node);
     Geometry::ChildList collectChildren3D(const AbstractNode &node);
     Polygon2d *applyMinkowski2D(const AbstractNode &node);
     Polygon2d *applyHull2D(const AbstractNode &node);
     Geometry *applyHull3D(const AbstractNode &node);
     void applyResize3D(class CGAL_Nef_polyhedron &N, const Vector3d &newsize, const Eigen::Matrix<bool,3,1> &autosize);
     Polygon2d *applyToChildren2D(const AbstractNode &node, OpenSCADOperator op);
     ResultObject applyToChildren3D(const AbstractNode &node, OpenSCADOperator op);
     ResultObject applyToChildren(const AbstractNode &node, OpenSCADOperator op);
     void addToParent(const State &state, const AbstractNode &node, std::shared_future<shared_ptr<const Geometry> > geom);

     std::map<int, ChildList> visitedchildren;
     const Tree &tree;
     std::mutex tree_mutex;
     shared_ptr<const Geometry> root;

public:
};

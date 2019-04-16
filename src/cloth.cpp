#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
  for (int j = 0; j < num_height_points; j++) {
    for (int i = 0; i < num_width_points; i++) {
      double width_unit = (double) width/ (double) num_width_points;
      double height_unit = (double) height/ (double) num_height_points;

      double x = (double) i * width_unit;
      double y = (double) j * height_unit;

      Vector3D location;

      if (orientation == 0)
        location = Vector3D(x,1.0,y);
      else
        location = Vector3D(x,y, -1.0/1000.0 + (double)rand() / RAND_MAX * (1.0/1000.0 - -1.0/1000.0));

      point_masses.emplace_back(PointMass(location, false));
    }
  }

  for (int i = 0; i < pinned.size(); i++) {
    vector<int> xy = pinned[i];
    int index = xy[0] + xy[1] * num_width_points;
    point_masses[index].pinned = true;
  }


  for (int j = 0; j < num_height_points; j++) {
    for (int i = 0; i < num_width_points; i++) {      
      if (i>0)
        springs.push_back(Spring(&point_masses[j*num_width_points + i - 1], &point_masses[j*num_width_points + i], STRUCTURAL));
      if (j>0)
        springs.push_back(Spring(&point_masses[(j-1)*num_width_points + i], &point_masses[j*num_width_points + i], STRUCTURAL));
    
      if (j>0 && i>0)
        springs.push_back(Spring(&point_masses[(j-1)*num_width_points + i-1], &point_masses[j*num_width_points + i], SHEARING));

      if (j>0 && (i+1)<num_width_points)
        springs.push_back(Spring(&point_masses[(j-1)*num_width_points + i+1], &point_masses[j*num_width_points + i], SHEARING));

      if ((i+2)<num_width_points)
        springs.push_back(Spring(&point_masses[j*num_width_points + i + 2], &point_masses[j*num_width_points + i], BENDING));

      if ((j-2)>=0)
        springs.push_back(Spring(&point_masses[(j-2)*num_width_points + i], &point_masses[j*num_width_points + i], BENDING));
    }
  }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
  for (PointMass& p : this->point_masses) {
    p.forces = Vector3D();
    for (Vector3D& a : external_accelerations)
      p.forces += mass * a;
  }
  for (Spring& s : this->springs) {
    if ((s.spring_type == STRUCTURAL && cp->enable_structural_constraints) ||
        (s.spring_type == SHEARING && cp->enable_shearing_constraints) ||
        (s.spring_type == BENDING && cp->enable_bending_constraints)) {
      PointMass* p1 = s.pm_a;
      PointMass* p2 = s.pm_b;
      Vector3D p_vec = p2->position - p1->position;
      double is_bending = s.spring_type == BENDING ? 0.2 : 1.0; 
      double F = is_bending * cp->ks * (p_vec.norm() - s.rest_length);
      p_vec.normalize();
      p1->forces += F * p_vec;
      p2->forces += -F * p_vec;
    }  
  }


  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for (PointMass& p : this->point_masses) {
    if (!p.pinned) {
      Vector3D newPos = p.position + (1.0 - cp->damping/100.0) * (p.position - p.last_position) + p.forces * delta_t * delta_t / mass;
      p.last_position = p.position;
      p.position = newPos;
    }
  }

  // TODO (Part 4): Handle self-collisions.
  build_spatial_map();
  for (PointMass &pm : point_masses) {
    self_collide(pm, simulation_steps);
  }


  // TODO (Part 3): Handle collisions with other primitives.
  for (PointMass &pm : point_masses) {
    for (CollisionObject *co : *collision_objects)
      co->collide(pm);
  }


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (Spring& s : this-> springs) {
    if ((s.spring_type == STRUCTURAL && cp->enable_structural_constraints) ||
        (s.spring_type == SHEARING && cp->enable_shearing_constraints) ||
        (s.spring_type == BENDING && cp->enable_bending_constraints)) {

      PointMass* m1 = s.pm_a;
      PointMass* m2 = s.pm_b;
      Vector3D p_vec = m2->position - m1->position;

      if (p_vec.norm() > s.rest_length * (1.0 + 0.1)) {
        p_vec.normalize();
        Vector3D shift = (1.0 + 0.1) * s.rest_length * p_vec;
        if (!m1->pinned && !m2->pinned) {
          Vector3D mp = (m1->position + m2->position) * 0.5;
          m1->position = -0.5 * shift + mp;
          m2->position = 0.5 * shift + mp;
        } 
        else if (!m1->pinned && m2->pinned)
          m1->position = -shift + m2->position;
        else if (m1->pinned && !m2->pinned)
          m2->position = shift + m1->position;
      }
    }  
  }

}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (PointMass &pm: point_masses) {
    double hash = hash_position(pm.position);
    if (map[hash] == NULL) {
      map[hash] = new vector<PointMass *>;
    }
    map[hash]->push_back(&pm);
  }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
  Vector3D correction = Vector3D();
  int num = 0;
  for (PointMass *pmc : *map[hash_position(pm.position)]) {
    if (&pm == pmc) continue;
    double diff = 2.0*thickness - ((pm.position - pmc->position).norm());
    if (diff < 0) continue;
    correction += diff * ((pm.position - pmc->position).unit());
    num++;
  }
  if (num == 0) return;
  correction /= num;
  correction /= simulation_steps;
  pm.position += correction;
  return;
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
  double w = 3 * width / num_width_points;
  double h = 3 * height / num_height_points;
  double t = max(w, h);

  float a = (pos.x - fmod(pos.x, w)) / w;
  float b = (pos.y - fmod(pos.y, h)) / h;
  float c = (pos.z - fmod(pos.z, t)) / t;

  return a + b * num_width_points + c * num_height_points * num_width_points;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}

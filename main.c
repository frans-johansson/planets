#include "raylib.h"
#include "raymath.h"
#include <stdio.h>
#include <stdlib.h>

#define WINDOW_WIDTH 1000
#define WINDOW_HEIGHT 1000
#define SYSTEM_INITIAL_CAPACITY 16
#define FORCES_INITIAL_CAPACITY 16
#define TRACE_INITIAL_CAPACITY 8192
#define GRAVITATIONAL_CONSTANT 1.0f
#define TEMPORARY_CHAR_BUFFER_SIZE 512
#define SPAWNER_VELOCITY_SCALE 0.1f
#define HUD_FONT_SIZE 24
#define HUD_PADDING 10
#define NUM_COLORS 6

Color color_get(size_t i) {
  return (Color[]){ORANGE, MAGENTA, BLUE, GRAY, YELLOW, PURPLE}[i % NUM_COLORS];
}

typedef enum {
  SUCCESS = 0,
  NOT_ENOUGH_MEMORY = 1,
} MemResult;

typedef struct {
  Vector2 *points;
  size_t count;
  size_t capacity;
} Trace;

MemResult trace_init(Trace *trace) {
  Vector2 *points = malloc(TRACE_INITIAL_CAPACITY * sizeof(Vector2));
  if (points == NULL) {
    printf("ERROR: Could no allocate memory for trace\n");
    return NOT_ENOUGH_MEMORY;
  }
  trace->points = points;
  trace->count = 0;
  trace->capacity = TRACE_INITIAL_CAPACITY;
  return SUCCESS;
}

MemResult trace_realloc(Trace *trace) {
  size_t new_capacity = trace->capacity * 2;
  Vector2 *points = realloc(trace->points, new_capacity * sizeof(Vector2));
  if (points == NULL) {
    printf("ERROR: Could not re-allocate memory for trace\n");
    return NOT_ENOUGH_MEMORY;
  }
  trace->points = points;
  trace->capacity = new_capacity;
  printf("INFO: Reallocating trace buffer\n");
  return SUCCESS;
}

void trace_append(Trace *trace, Vector2 point) {
  if (trace->count >= trace->capacity) {
    if (trace_realloc(trace) != SUCCESS) {
      printf("ERROR: Could not add point to trace\n");
      return;
    }
  }
  trace->points[trace->count++] = point;
}

void trace_free(Trace *trace) { free(trace->points); }

typedef struct {
  Vector2 *forces;
  size_t count;
  size_t capacity;
} Forces;

MemResult forces_init(Forces *forces) {
  Vector2 *forces_memory = malloc(FORCES_INITIAL_CAPACITY * sizeof(Vector2));
  if (forces_memory == NULL) {
    printf("ERROR: Could no allocate memory for forces\n");
    return NOT_ENOUGH_MEMORY;
  }
  forces->forces = forces_memory;
  forces->count = 0;
  forces->capacity = FORCES_INITIAL_CAPACITY;
  return SUCCESS;
}

MemResult forces_realloc(Forces *forces) {
  size_t new_capacity = forces->capacity * 2;
  Vector2 *forces_array =
      realloc(forces->forces, new_capacity * sizeof(Vector2));
  if (forces_array == NULL) {
    printf("ERROR: Could not re-allocate memory for forces\n");
    return NOT_ENOUGH_MEMORY;
  }
  forces->forces = forces_array;
  forces->capacity = new_capacity;
  return SUCCESS;
}

void forces_append(Forces *forces, Vector2 force) {
  if (forces->count >= forces->capacity) {
    if (forces_realloc(forces) != SUCCESS) {
      printf("ERROR: Could not add force to forces\n");
      return;
    }
  }
  forces->forces[forces->count++] = force;
}

void forces_free(Forces *forces) { free(forces->forces); }

#define ARROW_NUM_VERTICES 5
#define ARROW_WIDTH 15
typedef Vector2 Arrow[ARROW_NUM_VERTICES];

void arrow_at_origin(Arrow arrow, float length, float width) {
  /* Vertex layout:
   * 0 - 1 is a line
   * 2 - 4 is a triangle
   * 0 = origin
   *
   *  o--> (x)
   *  |
   *  v
   * (y)
   *             width
   *            <----->
   *          ^    3
   *          |   / \
   *   width  |  /   \
   *          v 4-.1.-2
   *          ^   | |
   *          |   | |
   *  length  |   | |
   *          |   | |
   *          v   .0.
   */
  arrow[0] = Vector2Zero();
  arrow[1] = Vector2Add(arrow[0], (Vector2){0.0f, -length});
  arrow[2] = Vector2Add(arrow[1], (Vector2){0.5f * width, 0.0f});
  arrow[3] = Vector2Add(arrow[1], (Vector2){0.0f, -width});
  arrow[4] = Vector2Add(arrow[1], (Vector2){-0.5f * width, 0.0f});
}

void arrow_from_vector(Arrow arrow, Vector2 vector) {
  float length = Vector2Length(vector);
  float angle = Vector2Angle((Vector2){0.0f, -1.0f}, vector);
  arrow_at_origin(arrow, length, ARROW_WIDTH);
  for (size_t i = 0; i < ARROW_NUM_VERTICES; ++i) {
    arrow[i] = Vector2Rotate(arrow[i], angle);
  }
}

void arrow_draw(Arrow arrow, Color color) {
  DrawLineV(arrow[0], arrow[1], color);
  DrawTriangle(arrow[2], arrow[3], arrow[4], color);
}

typedef struct {
  Vector2 position;
  Vector2 velocity;
  Vector2 acceleration;
  Forces forces;
  Trace trace;
  float radius;
  float mass;
  Color color;
  int is_selected;
} Body;

void body_init(Body *body) {
  forces_init(&body->forces);
  trace_init(&body->trace);
}

void body_free(Body *body) {
  forces_free(&body->forces);
  trace_free(&body->trace);
}

void body_apply_forces(Body *body) {
  Vector2 acceleration = {0};
  for (size_t i = 0; i < body->forces.count; ++i) {
    acceleration =
        Vector2Add(acceleration, Vector2Scale(body->forces.forces[i],
                                              1 / (EPSILON + body->mass)));
  }
  body->acceleration = acceleration;
}

void body_update(Body *body, float dt) {
  body_apply_forces(body);
  body->velocity =
      Vector2Add(body->velocity, Vector2Scale(body->acceleration, dt));
  body->position = Vector2Add(body->position, Vector2Scale(body->velocity, dt));
  trace_append(&body->trace, body->position);
  body->forces.count = 0; // Reset the forces array
}

void body_resolve_collisions(Body *this, Body *other) {
  float overlap = (this->radius + other->radius) -
                  Vector2Distance(this->position, other->position);
  if (overlap > 0.0f) {
    Vector2 direction =
        Vector2Normalize(Vector2Subtract(other->position, this->position));
    other->position =
        Vector2Add(other->position, Vector2Scale(direction, overlap));
  }
}

Vector2 body_compute_gravity(Body *this, Body *other) {
  Vector2 force_direction =
      Vector2Normalize(Vector2Subtract(other->position, this->position));

  float distance_sqr = Vector2DistanceSqr(this->position, other->position);
  float force_magnitude =
      GRAVITATIONAL_CONSTANT * this->mass * other->mass / distance_sqr;

  return Vector2Scale(force_direction, force_magnitude);
}

void body_draw(Body *body) {
  DrawCircleV(body->position, body->radius, body->color);
}

void body_draw_trace(Body *body) {
  for (size_t i = 0; i < body->trace.count; ++i) {
    DrawPixelV(body->trace.points[i], body->color);
  }
}

void body_draw_vector(Body *body, Vector2 vector) {
  Arrow arrow = {0};
  arrow_from_vector(arrow, Vector2ClampValue(vector, 0.0f, 512.0f));
  for (size_t i = 0; i < ARROW_NUM_VERTICES; ++i) {
    arrow[i] = Vector2Add(arrow[i], body->position);
  }
  arrow_draw(arrow, body->color);
}

void body_display_info(Body *body) {
  char info_text[TEMPORARY_CHAR_BUFFER_SIZE];
  snprintf(info_text, TEMPORARY_CHAR_BUFFER_SIZE,
           "Mass = %.2f\n\nAcc = (%.2f, %.2f)\n\nVel = (%.2f, %.2f)",
           body->mass, body->acceleration.x, body->acceleration.y,
           body->velocity.x, body->velocity.y);
  int info_x = (int)body->position.x + body->radius + HUD_PADDING;
  int info_y = (int)body->position.y + body->radius + HUD_PADDING;
  DrawText(info_text, info_x, info_y, HUD_FONT_SIZE, body->color);
}

typedef struct {
  Body *bodies;
  size_t count;
  size_t capacity;
} System;

MemResult system_init(System *system) {
  Body *bodies = malloc(SYSTEM_INITIAL_CAPACITY * sizeof(Body));
  if (bodies == NULL) {
    printf("ERROR: Could no allocate memory for system\n");
    return NOT_ENOUGH_MEMORY;
  }
  system->bodies = bodies;
  system->count = 0;
  system->capacity = SYSTEM_INITIAL_CAPACITY;
  return SUCCESS;
}

MemResult system_realloc(System *system) {
  size_t new_capacity = system->capacity * 2;
  Body *bodies = realloc(system->bodies, new_capacity * sizeof(Body *));
  if (bodies == NULL) {
    printf("ERROR: Could not re-allocate memory for system\n");
    return NOT_ENOUGH_MEMORY;
  }
  system->bodies = bodies;
  system->capacity = new_capacity;
  return SUCCESS;
}

void system_add_body(System *system, Body body) {
  if (system->count >= system->capacity) {
    if (system_realloc(system) != SUCCESS) {
      printf("ERROR: Could not add body to system\n");
      return;
    }
  }
  system->bodies[system->count++] = body;
}

void system_add_bodies(System *system, size_t count, Body bodies[count]) {
  for (size_t i = 0; i < count; ++i) {
    system_add_body(system, bodies[i]);
  }
}

void system_simulate(System *system) {
  for (size_t i = 0; i < system->count; ++i) {
    Body *this = &system->bodies[i];

    for (size_t j = 0; j < system->count; ++j) {
      if (i == j) {
        // Do not apply gravity between self and self
        continue;
      }

      Body *other = &system->bodies[j];
      forces_append(&this->forces, body_compute_gravity(this, other));
    }
  }
}

void system_update_bodies(System *system, float dt) {
  for (size_t i = 0; i < system->count; ++i) {
    body_update(&system->bodies[i], dt);
  }

  for (size_t i = 0; i < system->count; ++i) {
    for (size_t j = 0; j < system->count; ++j) {
      if (i == j) {
        continue;
      }
      body_resolve_collisions(&system->bodies[i], &system->bodies[j]);
    }
  }
}

void system_draw_bodies(System *system) {
  for (size_t i = 0; i < system->count; ++i) {
    body_draw(&system->bodies[i]);
    body_draw_trace(&system->bodies[i]);
  }
}

void system_draw_force_vectors(System *system) {
  for (size_t i = 0; i < system->count; ++i) {
    Body *body = &system->bodies[i];
    for (size_t j = 0; j < body->forces.count; ++j) {
      body_draw_vector(body, Vector2Scale(body->forces.forces[j], 1.0f));
    }
  }
}

void system_draw_acceleration_vectors(System *system) {
  for (size_t i = 0; i < system->count; ++i) {
    Body *body = &system->bodies[i];
    body_draw_vector(body, body->acceleration);
  }
}

void system_free(System *system) {
  for (size_t i = 0; i < system->count; ++i) {
    body_free(&system->bodies[i]);
  }
  free(system->bodies);
}

void system_init_default(System *system) {
  Body sun = {0};
  body_init(&sun);
  sun.position = (Vector2){WINDOW_WIDTH / 2.0f, WINDOW_HEIGHT / 2.0f};
  sun.radius = 100.0f;
  sun.mass = 10000.0f;
  sun.color = YELLOW;

  Body small_planet = {0};
  body_init(&small_planet);
  small_planet.position = Vector2Add(sun.position, (Vector2){0.0f, 300.0f});
  small_planet.velocity = (Vector2){5.0f, 0.0f};
  small_planet.radius = 10.0f;
  small_planet.mass = 1000.0f;
  small_planet.color = BLUE;

  system_init(system);
  system_add_bodies(system, 2, (Body[2]){sun, small_planet});
}

void system_select_body(System *system, Camera2D *camera) {
  Vector2 mouse_position = GetScreenToWorld2D(GetMousePosition(), *camera);
  for (size_t i = 0; i < system->count; ++i) {
    Body *this = &system->bodies[i];
    this->is_selected ^=
        CheckCollisionPointCircle(mouse_position, this->position, this->radius);
  }
}

void system_handle_input(System *system, Camera2D *camera) {
  if (IsKeyPressed(KEY_ENTER)) {
    system_free(system);
    system_init_default(system);
  }
  if (IsKeyPressed(KEY_BACKSPACE)) {
    system_free(system);
    system_init(system);
  }
  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    system_select_body(system, camera);
  }
}

typedef struct {
  int running;
  int spawn_mode_on;
  float time_scale;
} State;

void state_handle_input(State *state) {
  if (IsKeyPressed(KEY_SPACE)) {
    state->running = (state->running == 0) ? 1 : 0;
  }
  if (IsKeyPressed(KEY_UP)) {
    state->time_scale *= 2.0f;
  }
  if (IsKeyPressed(KEY_DOWN)) {
    state->time_scale *= 0.5f;
  }
  if (IsKeyPressed(KEY_Q)) {
    state->spawn_mode_on = (state->spawn_mode_on == 0) ? 1 : 0;
  }
}

void state_draw_hud(State *state) {
  DrawText((state->running == 0) ? "Paused" : "Running", HUD_PADDING,
           HUD_PADDING, HUD_FONT_SIZE, (state->running == 0) ? RED : GREEN);

  char time_scale_text[TEMPORARY_CHAR_BUFFER_SIZE] = {0};
  snprintf(time_scale_text, TEMPORARY_CHAR_BUFFER_SIZE, "Time scale: %.2f",
           state->time_scale);
  DrawText(time_scale_text, HUD_PADDING, HUD_PADDING + HUD_FONT_SIZE, 25,
           RAYWHITE);

  DrawText((state->spawn_mode_on != 0) ? "Spawn a new body" : "", HUD_PADDING,
           HUD_PADDING + HUD_FONT_SIZE * 2.0f, HUD_FONT_SIZE, RAYWHITE);
}

typedef struct {
  Vector2 position;
  Vector2 velocity;
  float radius;
  float mass;
  size_t color;
} Spawner;

Body spawner_make_body(Spawner *spawner) {
  Body body = {0};
  body_init(&body);
  body.color = color_get(spawner->color);
  body.position = spawner->position;
  body.velocity = Vector2Scale(spawner->velocity, SPAWNER_VELOCITY_SCALE);
  body.radius = spawner->radius;
  body.mass = spawner->mass;
  return body;
}

void spawner_reset(Spawner *spawner) { spawner->velocity = Vector2Zero(); }

void spawner_handle_input(Spawner *spawner, System *system, Camera2D *camera) {
  // Mouse input
  Vector2 mouse_position = GetScreenToWorld2D(GetMousePosition(), *camera);
  if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
    system_add_body(system, spawner_make_body(spawner));
    spawner_reset(spawner);
  }
  if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
    spawner->velocity =
        Vector2Rotate(Vector2Subtract(mouse_position, spawner->position), PI);
  } else {
    spawner->position = mouse_position;
  }

  // Keyboard input
  int ctrl_down = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
  float *value_to_modify = ctrl_down ? &spawner->radius : &spawner->mass;
  if (IsKeyPressed(KEY_LEFT)) {
    *value_to_modify *= 0.5f;
  } else if (IsKeyPressed(KEY_RIGHT)) {
    *value_to_modify *= 2.0f;
  }

  if (IsKeyPressed(KEY_PERIOD)) {
    ++spawner->color;
    spawner->color %= NUM_COLORS;
  }
  if (IsKeyPressed(KEY_COMMA)) {
    // FIXME: Modulo is broken when negative
    --spawner->color;
    spawner->color %= NUM_COLORS;
  }
}

void spawner_draw_preview_hud(Spawner *spawner) {
  Body preview = spawner_make_body(spawner);
  preview.color = ColorAlpha(preview.color, 0.25f);
  body_draw(&preview);
  if (Vector2LengthSqr(preview.velocity) > 0) {
    body_draw_vector(
        &preview, Vector2Scale(preview.velocity, 1 / SPAWNER_VELOCITY_SCALE));
  }

  char preview_text[TEMPORARY_CHAR_BUFFER_SIZE];
  snprintf(preview_text, TEMPORARY_CHAR_BUFFER_SIZE,
           "Radius = %.2f\n\nMass = %.2f\n\nColor = %ld", spawner->radius,
           spawner->mass, spawner->color);
  int preview_hud_x = (int)spawner->position.x + spawner->radius + HUD_PADDING;
  int preview_hud_y = (int)spawner->position.y + spawner->radius + HUD_PADDING;
  DrawText(preview_text, preview_hud_x, preview_hud_y, HUD_FONT_SIZE,
           color_get(spawner->color));
}

void system_display_body_info(System *system, Camera2D *camera) {
  Vector2 mouse_position = GetScreenToWorld2D(GetMousePosition(), *camera);
  for (size_t i = 0; i < system->count; ++i) {
    Body *this = &system->bodies[i];
    if (this->is_selected ||
        CheckCollisionPointCircle(mouse_position, this->position,
                                  this->radius)) {
      body_display_info(this);
    }
  }
}

void camera_handle_input(Camera2D *camera) {
  if (IsKeyDown(KEY_W)) {
    camera->offset =
        Vector2Add(camera->offset, Vector2Scale((Vector2){0.0f, -1.0f}, 2.0f));
  }
  if (IsKeyDown(KEY_A)) {
    camera->offset =
        Vector2Add(camera->offset, Vector2Scale((Vector2){-1.0f, 0.0f}, 2.0f));
  }
  if (IsKeyDown(KEY_S)) {
    camera->offset =
        Vector2Add(camera->offset, Vector2Scale((Vector2){0.0f, 1.0f}, 2.0f));
  }
  if (IsKeyDown(KEY_D)) {
    camera->offset =
        Vector2Add(camera->offset, Vector2Scale((Vector2){1.0f, 0.0f}, 2.0f));
  }
  if (IsKeyPressed(KEY_RIGHT_BRACKET)) {
    camera->zoom *= 1.2f;
  }
  if (IsKeyPressed(KEY_LEFT_BRACKET)) {
    camera->zoom *= 0.8f;
  }
}

int main(void) {
  InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Planets");
  SetTargetFPS(60);

  State state = {0};
  state.running = 0;
  state.time_scale = 1.0f;

  System system = {0};
  system_init(&system);

  Spawner spawner = {0};
  spawner.color = 0;
  spawner.radius = 20.0f;
  spawner.mass = 2000.0f;

  Camera2D camera = {0};
  camera.zoom = 1.0f;

  while (!WindowShouldClose()) {
    state_handle_input(&state);
    system_handle_input(&system, &camera);
    if (state.spawn_mode_on) {
      spawner_handle_input(&spawner, &system, &camera);
    }
    camera_handle_input(&camera);

    if (state.running) {
      // NOTE: Running the simulation step here sets up all force vectors
      // acting on each of the system's bodies. Separating this step from the
      // update step makes it possible to render the force vectors.
      system_simulate(&system);
    }

    BeginDrawing();
    BeginMode2D(camera);
    ClearBackground(BLACK);
    if (state.spawn_mode_on) {
      spawner_draw_preview_hud(&spawner);
    }
    system_display_body_info(&system, &camera);
    system_draw_bodies(&system);
    system_draw_force_vectors(&system);
    EndMode2D();
    // NOTE: Handle "sticky" UI outside of camera movement. It still needs to
    // come after the camera mode though, otherwise nothing is drawn at all.
    state_draw_hud(&state);
    EndDrawing();

    // NOTE: Breaks on higher time scales
    float dt = GetFrameTime() * state.time_scale;

    if (state.running) {
      // NOTE: Update handled after rendering in order to allow drawing force
      // vectors since the body_update function will clear all forces acting
      // on the body.
      system_update_bodies(&system, dt);
    }
  }

  CloseWindow();
}

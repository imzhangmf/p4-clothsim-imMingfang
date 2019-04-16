#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  return texture(u_texture_2, uv).x;
}

void main() {
  // YOUR CODE HERE
  mat3 TBN = mat3(v_tangent.xyz, cross(v_normal.xyz, v_tangent.xyz), v_normal.xyz);
  float kh = 1.0;
  float kn = u_normal_scaling;
  float dU = (h(v_uv + vec2(1/u_texture_2_size.x, 0)) - h(v_uv))*kh*kn;
  float dV = (h(v_uv + vec2(0, 1/u_texture_2_size.y)) - h(v_uv))*kh*kn;
  vec3 nd = TBN * vec3(-dU, -dV, 1.0);

  float ka = 0.15;
  vec4 Ia = vec4(1, 1, 1, 0);

  float kd = 0.5;
  vec4 I = vec4(u_light_intensity, 1.0);
  float r = length(vec4(u_light_pos, 1.0) - v_position);
  vec4 l = vec4(u_light_pos, 1.0) - v_position;

  float ks = 1.0;
  vec4 h = ((vec4(u_cam_pos, 1.0) - v_position) + l)/length((vec4(u_cam_pos, 1.0) - v_position) + l);
  int p = 5;
  // (Placeholder code. You will want to replace it.)
  // out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
  out_color = ka*Ia;
  out_color += kd*I/r/r*max(0, dot(vec4(nd, 1.0), normalize(l)));
  out_color += ks*I/r/r*pow(max(0, dot(vec4(nd, 1.0), normalize(h))), p);
  out_color.a = 1;
}


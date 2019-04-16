#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

// vec4 shadePhong() {
//   float p = 8.0;

//   vec4 color = u_color * 0.35;

//   vec3 lightVec = u_light_pos - v_position.xyz;
//   vec3 lightDir = normalize(lightVec);
//   vec3 outDir = normalize(u_cam_pos - v_position.xyz);
//   vec3 n = normalize(v_normal.xyz);

//   float distFactor = 1.0 / sqrt(dot(lightVec, lightVec));

//   vec4 ambient = color * 0.9;
//   // ambient.a = 0.5;

//   float diffuseDot = dot(n, lightDir);
//   vec4 diffuse = color * clamp(diffuseDot, 0.0, 1.0);

//   vec3 halfAngle = normalize(outDir + lightDir);
//   vec4 specularColor = min(color + 0.2, 1.0);
//   float specularDot = dot(n, halfAngle);
//   vec4 specular = 0.5 * specularColor * pow(clamp(specularDot, 0.0, 1.0), p);

//   return diffuse + ambient + specular;
// }

void main() {
  // YOUR CODE HERE
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
  out_color = ka*Ia + kd*I/(r*r)*max(0, dot(v_normal, normalize(l)));
  out_color += ks*(I/r/r)*pow(max(0, dot(v_normal, normalize(h))),p);
  out_color.a = u_color.a;
  // out_color = shadePhong();
  // out_color.a = u_color.a;
}


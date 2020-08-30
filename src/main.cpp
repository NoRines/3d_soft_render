#include "input/controller_input.hpp"
#include "input/event_handling.hpp"
#include "input/keyboard_input.hpp"
#include "input/window_state.hpp"
#include "math/mat_math.hpp"
#include "math/vec_math.hpp"
#include "triangle_drawer.hpp"
#include "util.hpp"
#include <iostream>

#include <iomanip>

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include <SDL2/SDL.h>

struct Vertex {
  Vec3f pos;
  Vec3f color;
  Vec2f texCoord;
};

using Face = std::array<Vertex, 3>;
using Mesh = std::vector<Face>;

std::ostream &operator<<(std::ostream &os, const Vec4f &v) {
  os << v.x() << ", " << v.y() << ", " << v.z() << ", " << v.w();
  return os;
}
std::ostream &operator<<(std::ostream &os, const Vec3f &v) {
  os << v.x() << ", " << v.y() << ", " << v.z();
  return os;
}
std::ostream &operator<<(std::ostream &os, const Vec2f &v) {
  os << v.x() << ", " << v.y();
  return os;
}

std::optional<std::array<Vec4f, 3>>
faceToCameraSpace(const Vec3f &pos0, const Vec3f &pos1, const Vec3f &pos2,
                  const Mat4f &worldViewM) {
  // Apply the world and view matrix
  const auto v0 = worldViewM * toVec4(pos0, 1.0f);
  const auto v1 = worldViewM * toVec4(pos1, 1.0f);
  const auto v2 = worldViewM * toVec4(pos2, 1.0f);

  const auto viewNormal =
      (toVec3(v1) - toVec3(v0)).cross(toVec3(v2) - toVec3(v0)).unit();

  if (toVec3(v0).unit().dot(viewNormal) > 0.0f) {
    return std::nullopt;
  }

  return {{v0, v1, v2}};
}

std::array<Vec4f, 3> cameraSpaceToDeviceCoords(const Vec4f &pos0,
                                               const Vec4f &pos1,
                                               const Vec4f &pos2,
                                               const Mat4f &projM) {

  // Apply the projection matrix
  const auto p0 = projM * pos0;
  const auto p1 = projM * pos1;
  const auto p2 = projM * pos2;

  // Do perspective divide
  const auto r0 = toVec4(toVec3(p0), 1.0f) / p0.w();
  const auto r1 = toVec4(toVec3(p1), 1.0f) / p1.w();
  const auto r2 = toVec4(toVec3(p2), 1.0f) / p2.w();

  return {r0, r1, r2};
}

std::array<Vec2f, 3>
deviceCoordsToScreenCoords(const Vec4f &v0, const Vec4f &v1, const Vec4f &v2,
                           int halfScreenWidth, int halfScreenHeight) {
  const Vec2f s0(halfScreenWidth * (v0.x() + 1.0f),
                 (halfScreenHeight * (-v0.y() + 1.0f)));
  const Vec2f s1(halfScreenWidth * (v1.x() + 1.0f),
                 (halfScreenHeight * (-v1.y() + 1.0f)));
  const Vec2f s2(halfScreenWidth * (v2.x() + 1.0f),
                 (halfScreenHeight * (-v2.y() + 1.0f)));

  return {s0, s1, s2};
}

float distToPlaneLineIntersection(const Vec3f &planeP, const Vec3f &planeN,
                                  const Vec3f &p0, const Vec3f &p1) {
  return planeN.dot(planeP - p0) / planeN.dot(p1 - p0);
}

bool classifyPointToPlane(const Vec3f &planeP, const Vec3f &planeN,
                          const Vec3f &p) {
  return planeN.dot(p - planeP) > 0.0f;
}

auto classifyFaceToPlane(const Vec3f &planeP, const Vec3f &planeN,
                         const Vec3f &p0, const Vec3f &p1, const Vec3f &p2) {
  int numPointsFront = 3;
  std::array<int, 3> indexes = {0, 0, 0};
  numPointsFront -= !classifyPointToPlane(planeP, planeN, p0);
  indexes[3 - numPointsFront] = 1;
  numPointsFront -= !classifyPointToPlane(planeP, planeN, p1);
  indexes[3 - numPointsFront] = 2;
  numPointsFront -= !classifyPointToPlane(planeP, planeN, p2);
  return std::make_pair(3 - numPointsFront, indexes);
}

template <typename T> concept is_vert = is_tuple<T> and requires(T t) {
  {toVec3(std::get<0>(t))};
};

template <is_vert T>
std::array<std::optional<std::array<T, 3>>, 2>
clipFaceToPlane(const Vec3f &planeP, const Vec3f &planeN, const T &f0,
                const T &f1, const T &f2) {

  constexpr std::size_t tSize = std::tuple_size_v<T>;

  auto lerpVertex = [](float t, const T &a, const T &b) {
    return [&]<std::size_t... i>(std::index_sequence<i...>) {
      return T{std::get<i>(a) + (std::get<i>(b) - std::get<i>(a)) * t...};
    }
    (std::make_index_sequence<tSize>());
  };

  const auto f = std::array{f0, f1, f2};

  // Classify the face
  const auto [numPointsBack, backIndexes] = classifyFaceToPlane(
      planeP, planeN, toVec3(std::get<0>(std::get<0>(f))),
      toVec3(std::get<0>(std::get<1>(f))), toVec3(std::get<0>(std::get<2>(f))));

  if (numPointsBack == 1) {
    // Make two new faces & return
    const std::size_t i0 = backIndexes[0];
    const std::size_t i1 = (i0 == 2) ? 0 : i0 + 1;
    const std::size_t i2 = (i1 == 2) ? 0 : i1 + 1;

    const float s0 = distToPlaneLineIntersection(
        planeP, planeN, toVec3(std::get<0>(f[i1])), toVec3(std::get<0>(f[i0])));
    const float s1 = distToPlaneLineIntersection(
        planeP, planeN, toVec3(std::get<0>(f[i2])), toVec3(std::get<0>(f[i0])));

    const auto v0 = lerpVertex(s0, f[i1], f[i0]);
    const auto v1 = lerpVertex(s1, f[i2], f[i0]);

    return {std::array<T, 3>{f[i1], f[i2], v0},
            std::array<T, 3>{v0, f[i2], v1}};
  } else if (numPointsBack == 2) {
    // Make one new face & return
    const std::size_t i0 = backIndexes[0];
    const std::size_t i1 = backIndexes[1];
    const std::size_t i2 = 3 - (i0 + i1);

    const float s0 = distToPlaneLineIntersection(
        planeP, planeN, toVec3(std::get<0>(f[i2])), toVec3(std::get<0>(f[i0])));
    const float s1 = distToPlaneLineIntersection(
        planeP, planeN, toVec3(std::get<0>(f[i2])), toVec3(std::get<0>(f[i1])));

    const auto v0 = lerpVertex(s0, f[i2], f[i0]);
    const auto v1 = lerpVertex(s1, f[i2], f[i1]);

    return {std::array<T, 3>{v0, v1, f[i2]}, std::nullopt};
  } else if (numPointsBack == 3) {
    return {std::nullopt, std::nullopt};
  }

  // No points back return
  return {f, std::nullopt};
}

std::uint32_t mixColors(std::uint32_t c0, std::uint32_t c1) {
  const uint32_t alpha = c0 >> 24;
  const std::uint32_t rb1 = ((0x100 - alpha) * (c1 & 0xFF00FF)) >> 8;
  const std::uint32_t rb2 = (alpha * (c0 & 0xFF00FF)) >> 8;
  const std::uint32_t g1 = ((0x100 - alpha) * (c1 & 0x00FF00)) >> 8;
  const std::uint32_t g2 = (alpha * (c0 & 0x00FF00)) >> 8;
  return (alpha << 24) | (((rb1 | rb2) & 0xFF00FF) + ((g1 | g2) & 0x00FF00));
}

class Bitmap {
public:
  Bitmap(int w, int h) : width(w), height(h), pixels(w * h) {}

  int getWidth() const { return width; }
  int getHeight() const { return height; }

  void clear(std::uint32_t c = 0) {
    std::fill(pixels.begin(), pixels.end(), c);
  }

  void setPixel(int x, int y, std::uint32_t c) { pixels[x + y * width] = c; }
  std::uint32_t getPixel(int x, int y) const { return pixels[x + y * width]; }

  const std::uint8_t *get() const {
    return reinterpret_cast<const std::uint8_t *>(pixels.data());
  }

private:
  int width;
  int height;
  std::vector<std::uint32_t> pixels;
};

template <is_vert T>
void recursiveFaceClipAndDraw(T &&f0, T &&f1, T &&f2, Bitmap &bitmap,
                              const Bitmap &randomTex,
                              std::size_t planeIndex = 0) {
  using Plane = std::pair<Vec3f, Vec3f>;
  constexpr std::array<Plane, 4> clipPlanes{
      Plane({1.0f, 0.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}), // left
      Plane({0.0f, -1.0f, 0.0f}, {0.0f, 1.0f, 0.0f}), // top
      Plane({-1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}), // right
      Plane({0.0f, 1.0f, 0.0f}, {0.0f, -1.0f, 0.0f}), // bottom
  };

  if (planeIndex == 4) {

    const auto &[nd0, col0, tex0] = f0;
    const auto &[nd1, col1, tex1] = f1;
    const auto &[nd2, col2, tex2] = f2;

    const auto [s0, s1, s2] = deviceCoordsToScreenCoords(
        nd0, nd1, nd2, bitmap.getWidth() / 2, bitmap.getHeight() / 2);

    const auto vert0 = std::tuple_cat(
        mf::util::convert_array_to<int>(s0.data),
        std::tuple(1.0f / nd0.w(), col0 / nd0.w(), tex0 / nd0.w()));
    const auto vert1 = std::tuple_cat(
        mf::util::convert_array_to<int>(s1.data),
        std::tuple(1.0f / nd1.w(), col1 / nd1.w(), tex1 / nd1.w()));
    const auto vert2 = std::tuple_cat(
        mf::util::convert_array_to<int>(s2.data),
        std::tuple(1.0f / nd2.w(), col2 / nd2.w(), tex2 / nd2.w()));

    drawTriangle<0, 1, 2, true>(
        vert0, vert1, vert2,
        [&bitmap, &randomTex](int x, int y, [[maybe_unused]] float z,
                              const Vec3f &c, const Vec2f &tc) {
          std::uint32_t color = 0x7F000000 |
                                (std::uint32_t)(c.r() * 255.0f) << 16 |
                                (std::uint32_t)(c.g() * 255.0f) << 8 |
                                (std::uint32_t)(c.b() * 255.0f);
          std::uint32_t texColor =
              randomTex.getPixel((int)(randomTex.getWidth() * tc.u()),
                                 (int)(randomTex.getHeight() * tc.v()));
          bitmap.setPixel(x, y, mixColors(texColor, color));
        });
    return;
  }

  auto [nf0, nf1] = clipFaceToPlane(clipPlanes[planeIndex].second,
                                    clipPlanes[planeIndex].first, f0, f1, f2);

  if (auto fp = nf0) {
    auto &[fp0, fp1, fp2] = fp.value();
    recursiveFaceClipAndDraw(fp0, fp1, fp2, bitmap, randomTex, planeIndex + 1);
  }
  if (auto fp = nf1) {
    auto &[fp0, fp1, fp2] = fp.value();
    recursiveFaceClipAndDraw(fp0, fp1, fp2, bitmap, randomTex, planeIndex + 1);
  }
}

struct SDLDestroyer {
  void operator()(SDL_Window *w) { SDL_DestroyWindow(w); }
  void operator()(SDL_Renderer *r) { SDL_DestroyRenderer(r); }
  void operator()(SDL_Texture *t) { SDL_DestroyTexture(t); }
};

int main([[maybe_unused]] int argc, [[maybe_unused]] const char **argv) {

  constexpr int width = 640;
  constexpr int height = 480;

  SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER);

  /* Run scope */ {
    std::unique_ptr<SDL_Window, SDLDestroyer> window{SDL_CreateWindow(
        "3d_soft_render", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width,
        height, SDL_WINDOW_RESIZABLE)};
    if (!window) {
      return 1;
    }

    std::unique_ptr<SDL_Renderer, SDLDestroyer> renderer{
        SDL_CreateRenderer(window.get(), -1, 0)};
    if (!renderer) {
      return 2;
    }

    std::unique_ptr<SDL_Texture, SDLDestroyer> texture{
        SDL_CreateTexture(renderer.get(), SDL_PIXELFORMAT_ARGB8888,
                          SDL_TEXTUREACCESS_STREAMING, width, height)};
    if (!texture) {
      return 3;
    }

    /* Handle start events */ {
      SDL_Event event;
      while (SDL_PollEvent(&event)) {
        mf::evnt::handleEvent(&event);
      }
    }

    Bitmap bitmap(width, height);
    Bitmap randomTex(10, 10);

    for (int y = 0; y < randomTex.getHeight(); y++) {
      for (int x = 0; x < randomTex.getWidth(); x++) {
        randomTex.setPixel(x, y, 0x44000000 | (rand() % 0x00FFFFFF));
      }
    }

    const Mesh m = {
        {{
            {{0.0f, 0.5f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.5f, 1.0f}},    // T
            {{-0.5f, -0.5f, -0.5f}, {0.0f, 1.0f, 1.0f}, {0.0f, 0.0f}}, // BR
            {{0.5f, -0.5f, -0.5f}, {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f}},  // BL
        }},
        {{
            {{0.0f, 0.5f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.5, 1.0f}},    // T
            {{0.5f, -0.5f, 0.5f}, {1.0f, 0.0f, 1.0f}, {0.0f, 0.0f}},  // TL
            {{-0.5f, -0.5f, 0.5f}, {1.0f, 1.0f, 0.0f}, {1.0f, 0.0f}}, // TR
        }},
        {{
            {{0.0f, 0.5f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.5, 1.0f}},    // T
            {{0.5f, -0.5f, -0.5f}, {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f}}, // BL
            {{0.5f, -0.5f, 0.5f}, {1.0f, 0.0f, 1.0f}, {0.0f, 0.0f}},  // TL
        }},
        {{
            {{0.0f, 0.5f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.5, 1.0f}},     // T
            {{-0.5f, -0.5f, 0.5f}, {1.0f, 1.0f, 0.0f}, {1.0f, 0.0f}},  // TR
            {{-0.5f, -0.5f, -0.5f}, {0.0f, 1.0f, 1.0f}, {0.0f, 0.0f}}, // BR
        }},
        {{
            {{-0.5f, -0.5f, -0.5f}, {0.0f, 1.0f, 1.0f}, {0.0f, 0.0f}}, // BR
            {{0.5f, -0.5f, 0.5f}, {1.0f, 0.0f, 1.0f}, {1.0f, 1.0f}},   // TL
            {{0.5f, -0.5f, -0.5f}, {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f}},  // BL
        }},
        {{
            {{-0.5f, -0.5f, -0.5f}, {0.0f, 1.0f, 1.0f}, {0.0f, 0.0f}}, // BR
            {{-0.5f, -0.5f, 0.5f}, {1.0f, 1.0f, 0.0f}, {0.0f, 1.0f}},  // TR
            {{0.5f, -0.5f, 0.5f}, {1.0f, 0.0f, 1.0f}, {1.0f, 1.0f}},   // TL
        }},
    };

    float rotAmt = 0.0f;

    constexpr float fovRad = 1.5708f;
    const auto projM =
        projectionMat((float)width / height, fovRad, 0.1f, 10.0f);

    Vec3f cameraEye(0.0f, 0.0f, 0.0f);
    Vec3f cameraDir(0.0f, 0.0f, 1.0f);
    constexpr Vec3f worldUp(0.0f, 1.0f, 0.0f);

    int controller = mf::cinpt::getController().value_or(-1);

    while (!mf::wstat::closed()) {
      const auto ticks = mf::util::limit_fps<std::chrono::microseconds, 60>();
      const float delta = static_cast<float>(ticks / 1000000.0);

      mf::kinpt::updateOldKeyMap();
      mf::cinpt::updateOldButtonMaps();

      SDL_Event event;
      while (SDL_PollEvent(&event)) {
        mf::evnt::handleEvent(&event);
      }

      bitmap.clear(0xFF00008B);

      /* Update and render */ {

        /* Move Camera */ {
          if (mf::kinpt::keyDown(SDLK_LSHIFT)) {
            if (mf::kinpt::keyDown(SDLK_UP)) {
              cameraEye += cameraDir.cross(worldUp) * delta;
            }
            if (mf::kinpt::keyDown(SDLK_DOWN)) {
              cameraEye -= cameraDir.cross(worldUp) * delta;
            }
          } else {
            if (mf::kinpt::keyDown(SDLK_UP)) {
              cameraEye += cameraDir * delta;
            }
            if (mf::kinpt::keyDown(SDLK_DOWN)) {
              cameraEye -= cameraDir * delta;
            }
          }

          if (mf::cinpt::buttonDown(controller,
                                    SDL_CONTROLLER_BUTTON_DPAD_UP)) {
            cameraEye += cameraDir * delta;
          } else if (mf::cinpt::buttonDown(controller,
                                           SDL_CONTROLLER_BUTTON_DPAD_DOWN)) {
            cameraEye -= cameraDir * delta;
          }
          if (mf::cinpt::buttonDown(controller,
                                    SDL_CONTROLLER_BUTTON_DPAD_LEFT)) {
            cameraEye -= cameraDir.cross(worldUp) * delta;
          } else if (mf::cinpt::buttonDown(controller,
                                           SDL_CONTROLLER_BUTTON_DPAD_RIGHT)) {
            cameraEye += cameraDir.cross(worldUp) * delta;
          }
        }

        /* Rotate Camera */ {
          Vec2f pitchYaw(0.0f, 0.0f);
          if (mf::kinpt::keyDown(SDLK_LEFT) ||
              (mf::cinpt::buttonDown(controller, SDL_CONTROLLER_BUTTON_A) &&
               !mf::cinpt::buttonDown(controller,
                                      SDL_CONTROLLER_BUTTON_BACK))) {
            pitchYaw.y() = delta;
          }
          if (mf::kinpt::keyDown(SDLK_RIGHT) ||
              (mf::cinpt::buttonDown(controller, SDL_CONTROLLER_BUTTON_B) &&
               !mf::cinpt::buttonDown(controller,
                                      SDL_CONTROLLER_BUTTON_BACK))) {
            pitchYaw.y() = -delta;
          }
          if (mf::kinpt::keyDown(SDLK_a) ||
              (mf::cinpt::buttonDown(controller, SDL_CONTROLLER_BUTTON_A) &&
               mf::cinpt::buttonDown(controller, SDL_CONTROLLER_BUTTON_BACK))) {
            pitchYaw.x() = delta;
          }
          if (mf::kinpt::keyDown(SDLK_s) ||
              (mf::cinpt::buttonDown(controller, SDL_CONTROLLER_BUTTON_B) &&
               mf::cinpt::buttonDown(controller, SDL_CONTROLLER_BUTTON_BACK))) {
            pitchYaw.x() = -delta;
          }

          const Vec3f pitchVec = cameraDir.cross(worldUp).unit();
          const Vec3f yawVec = pitchVec.cross(cameraDir).unit();

          cameraDir = toVec3(
              rotationMat4((pitchVec * pitchYaw.x() + yawVec * pitchYaw.y())) *
              toVec4(cameraDir, 0.0f));
        }

        // rotAmt += delta;

        // Calculate the world matrix
        const auto worldM = translateMat4(0.0f, 0.0f, 2.0f) *
                            scaleMat4(1.0f, 1.0f, 1.0f, 1.0f) *
                            rotationMat4(rotAmt, 0.5f * rotAmt, 1.4f * rotAmt);
        // Calculate view matrix
        const auto viewM = lookAtMat(cameraEye, cameraEye + cameraDir, worldUp);

        for (const auto &f : m) {

          if (const auto optCameraSpace = faceToCameraSpace(
                  f[0].pos, f[1].pos, f[2].pos, viewM * worldM)) {
            const auto &[c0, c1, c2] = optCameraSpace.value();

            // Clip with near plane
            auto newFaces = clipFaceToPlane(
                {0.0f, 0.0f, 0.1f}, {0.0f, 0.0f, 1.0f},
                std::tuple(toVec3(c0), f[0].color, f[0].texCoord),
                std::tuple(toVec3(c1), f[1].color, f[1].texCoord),
                std::tuple(toVec3(c2), f[2].color, f[2].texCoord));

            std::size_t i = 0;
            while (newFaces[i] && i < newFaces.size()) {

              const auto &[cVert0, cVert1, cVert2] = newFaces[i++].value();
              const auto &[cc0, col0, tex0] = cVert0;
              const auto &[cc1, col1, tex1] = cVert1;
              const auto &[cc2, col2, tex2] = cVert2;

              const auto [nd0, nd1, nd2] = cameraSpaceToDeviceCoords(
                  toVec4(cc0, 1.0f), toVec4(cc1, 1.0f), toVec4(cc2, 1.0f),
                  projM);

              recursiveFaceClipAndDraw(
                  std::tuple(nd0, col0 * nd0.w(), tex0 * nd0.w()),
                  std::tuple(nd1, col1 * nd1.w(), tex1 * nd1.w()),
                  std::tuple(nd2, col2 * nd2.w(), tex2 * nd2.w()), bitmap,
                  randomTex);
            }
          }
        }
      }

      SDL_UpdateTexture(texture.get(), nullptr, (const void *)bitmap.get(),
                        bitmap.getWidth() * 4);
      SDL_RenderClear(renderer.get());
      SDL_RenderCopy(renderer.get(), texture.get(), nullptr, nullptr);
      SDL_RenderPresent(renderer.get());
    }
  }

  SDL_Quit();

  return 0;
}

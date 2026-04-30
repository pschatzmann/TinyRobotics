#pragma once
#include <TinyGPU.h>  // https://pschatzmann.github.io/TinyGPU
#include <TinyRobotics/maps/GridMap.h>

namespace tinyrobotics {

/**
 * @class GridMapDisplay
 * @ingroup communication
 * @brief A class that visualizes a GridMap using the TinyGPU display API.
 * This class provides a way to visualize the state of a GridMap on a TinyGPU
 * display. It sets up a callback to update the display whenever a cell state
 * changes, and allows for customization of the color mapping and vehicle
 * sprite.
 * @note Dependency: https://pschatzmann.github.io/TinyGPU
 */

template <typename RGB_T = RGB565, typename StateT = CellState,
          typename T = DistanceM>
class GridMapDisplay {
 public:
  GridMapDisplay(GridMap<StateT, T>& map, uint32_t widthPixels,
                 uint32_t heightPixels)
      : p_gridMap(&map),
        widthPixels(widthPixels),
        heightPixels(heightPixels),
        deltaXPixels(0),
        deltaYPixels(0),
        getColorCB(nullptr),
        p_sprite_info(nullptr) {
    deltaXPixels = widthPixels / map.getXCount();
    deltaYPixels = heightPixels / map.getYCount();
  }

  /// Initialize the GPU and set up callbacks
  bool begin() {
    framebuffer.setFont(font);
    framebuffer.resize(widthPixels, heightPixels);
    if (!framebuffer.begin()) {
      // Serial.println("Failed to initialize TinyGPU framebuffer");
      return false;
    }

    // setup sprite for the vehicle (default is a simple arrow shape)
    if (vehicleSprite.getWidth() == 0 || vehicleSprite.getHeight() == 0) {
      vehicleSprite.resize(8, 9);  // Default size for the vehicle sprite
      vehicleSprite.begin();
      vehicleSprite.drawLine(3, 1, 4, 1, spriteColor);
      vehicleSprite.drawLine(2, 2, 5, 2, spriteColor);
      vehicleSprite.drawLine(1, 3, 6, 3, spriteColor);
      vehicleSprite.fillRect(3, 4, 2, 4, spriteColor);
    }

    // Set the callback to update the GPU whenever a cell state changes
    p_gridMap->setCellUpdateCallback([this](GridMap<StateT, T>& map, int cx,
                                            int cy, const Coordinate<T>& coord,
                                            CellState state) {
      this->updateGridMap(map, cx, cy, coord, state);
    });
    updateGPU();
    return true;
  }

  /// Clean up resources and remove callbacks
  void end() {
    framebuffer.end();
    // Remove callback to avoid dangling pointer
    p_gridMap->setCellUpdateCallback(nullptr);
  }

  /// Set the position and orientation of the vehicle on the grid map
  void displayVehicleAt(T x, T y, float angleDegrees) {
    // Account for grid origin
    T relX = x - p_gridMap->getOriginX();
    T relY = y - p_gridMap->getOriginY();
    int posX = static_cast<int>(relX / p_gridMap->getResolution());
    int posY = static_cast<int>(relY / p_gridMap->getResolution());
    // Center the sprite in the cell
    float pixelX = (posX + 0.5f) * deltaXPixels;
    float pixelY = (posY + 0.5f) * deltaYPixels;
    if (p_sprite_info == nullptr) {
      p_sprite_info = &framebuffer.addSprite(
          pixelX, pixelY, vehicleSprite, getColorForState(CellState::OCCUPIED));
    } else {
      p_sprite_info->setPosition(pixelX, pixelY);
    }
    p_sprite_info->setRotation(angleDegrees);
  }

  /// Define a custom color mapping for cell states (optional)
  void setColorCallback(RGB_T (*cb)(StateT state)) { getColorCB = cb; }

  /// Define a custom sprite for the vehicle (optional)
  void setSprite(const Sprite<RGB_T>& newSprite) {
    vehicleSprite = newSprite;
    // If the sprite is already on the GPU, update it
    if (p_sprite_info != nullptr) {
      float x = p_sprite_info->getX();
      float y = p_sprite_info->getY();
      float angle = p_sprite_info->getRotation();
      framebuffer.removeSprite(p_sprite_info);
      p_sprite_info = &framebuffer.addSprite(
          x, y, vehicleSprite, getColorForState(CellState::OCCUPIED));
      p_sprite_info->setRotation(angle);
    }
  }
  /// Defines the color used for the vehicle sprite (optional)
  void setSpriteColor(RGB_T color) { spriteColor = color; }

  /// Access the underlying TinyGPU framebuffer for direct drawing (optional)
  FrameBuffer<RGB_T>& getFrameBuffer() { return framebuffer; }

 protected:
  BitmapFont<RGB_T> font;
  GridMap<StateT, T>* p_gridMap;
  FrameBuffer<RGB_T> framebuffer;
  uint32_t widthPixels;   // Display width in pixels
  uint32_t heightPixels;  // Display height in pixels
  uint32_t deltaXPixels;  // Pixels per grid cell in X direction
  uint32_t deltaYPixels;  // Pixels per grid cell in Y direction
  RGB_T (*getColorCB)(StateT state);
  RGB_T spriteColor{0, 0, 0};  // Default sprite color (black)
  Sprite<RGB_T> vehicleSprite;
  FrameBuffer<RGB_T>::SpriteInfo* p_sprite_info;

  RGB_T getColorForState(StateT state) {
    // If a custom color callback is set, use it to get the color for the state
    if (getColorCB != nullptr) {
      return getColorCB(state);
    }
    // Default color mapping for CellState
    if constexpr (std::is_same<StateT, CellState>::value) {
      switch (state) {
        case CellState::FREE:
          return RGB_T(0, 255, 0);  // Green
        case CellState::OCCUPIED:
          return RGB_T(255, 0, 0);  // Red
        case CellState::UNKNOWN:
          return RGB_T(128, 128, 128);  // Gray
        default:
          return RGB_T(0, 0, 0);  // Black for invalid state
      }
    }
  }

  /// update the entire GPU grid map based on the current state of the GridMap
  void updateGPU() {
    // For each cell in the grid map, update the corresponding pixels in the GPU
    for (int y = 0; y < p_gridMap->getYCount(); ++y) {
      for (int x = 0; x < p_gridMap->getXCount(); ++x) {
        CellState state;
        if (p_gridMap->getCell(x, y, state)) {
          updateGridMap(*p_gridMap, x, y, p_gridMap->toWorld(x, y), state);
        }
      }
    }
  }

  /// Callback function to update the GPU when a cell state changes
  void updateGridMap(GridMap<StateT, T>& map, int cx, int cy,
                     const Coordinate<T>& coord, CellState state) {
    RGB_T color = getColorForState(state);
    int px = cx * deltaXPixels;
    int py = cy * deltaYPixels;
    // Fill the cell with the appropriate color
    framebuffer.fillRect(px, py, deltaXPixels, deltaYPixels, color);
    // Draw border (black)
    framebuffer.drawRect(px, py, deltaXPixels, deltaYPixels, RGB_T(0, 0, 0));
  }
};

}  // namespace tinyrobotics
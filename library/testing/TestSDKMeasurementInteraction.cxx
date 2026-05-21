#include "PseudoUnitTest.h"

#include <engine.h>
#include <interactor.h>
#include <scene.h>
#include <window.h>

#include <cstdlib>

int TestSDKMeasurementInteraction(int argc, char* argv[])
{
  const std::string dataPath = std::string(argv[1]) + "/data/cow.vtp";

  f3d::engine eng = f3d::engine::create(true);
  eng.getWindow().setSize(300, 300);
  eng.getScene().add(dataPath);

  f3d::interactor& inter = eng.getInteractor();

  inter.triggerCommand("toggle_measurement");

  inter.triggerMousePosition(120, 150);
  inter.triggerMouseButton(
    f3d::interactor::InputAction::PRESS, f3d::interactor::MouseButton::LEFT);
  inter.triggerMouseButton(
    f3d::interactor::InputAction::RELEASE, f3d::interactor::MouseButton::LEFT);

  inter.triggerMousePosition(190, 150);
  inter.triggerMouseButton(
    f3d::interactor::InputAction::PRESS, f3d::interactor::MouseButton::LEFT);
  inter.triggerMouseButton(
    f3d::interactor::InputAction::RELEASE, f3d::interactor::MouseButton::LEFT);

  eng.getWindow().render();

  inter.triggerKeyboardKey(f3d::interactor::InputAction::PRESS, "Escape");

  eng.getWindow().render();

  return EXIT_SUCCESS;
}

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>

#include <cstdio>

#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

// we want assert statements to work in release mode
#undef NDEBUG

namespace {
std::string exampleMapPath = "assets/sample-map-planning/lanelet2_map.osm";
std::string tempfile(const std::string& name) {
  char tmpDir[] = "/tmp/lanelet2_example_XXXXXX";
  auto* file = mkdtemp(tmpDir);
  if (file == nullptr) {
    throw lanelet::IOError("Failed to open a temporary file for writing");
  }
  return std::string(file) + '/' + name;
}

}

void part1LoadingAndWriting();
void part2Projectors();
// void part3AddingNewParsersAndWriters();

int main() {
  part1LoadingAndWriting();
  part2Projectors();
  // part3AddingNewParsersAndWriters();
  return 0;
}

void part1LoadingAndWriting() {
  using namespace lanelet;

  projection::UtmProjector projector(Origin({35.23808753540768, 139.9009591876285}));
  LaneletMapPtr map = load(exampleMapPath, projector);

  write(tempfile("map.bin"), *map);

  ErrorMessages errors;
  map = load(exampleMapPath, projector, &errors);
  assert(errors.empty());
}

void part2Projectors() {
  using namespace lanelet;

  projection::UtmProjector projector(Origin({35.23808753540768, 139.9009591876285}));
  BasicPoint3d projection = projector.forward(GPSPoint{35.23808753540768, 139.9009591876285, 0});
  assert(std::abs(projection.x()) < 1e-6);
}
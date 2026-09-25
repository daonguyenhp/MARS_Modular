#include "mode1_map.hpp"

#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace mars::demo_simulation {
namespace {

std::string trim(std::string text) {
  const auto begin = text.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return {};
  }
  const auto end = text.find_last_not_of(" \t\r\n");
  return text.substr(begin, end - begin + 1);
}

std::string directory_of(const std::string& path) {
  const auto slash = path.find_last_of("/\\");
  if (slash == std::string::npos) {
    return {};
  }
  return path.substr(0, slash + 1);
}

std::string join_image(const std::string& yaml_path, std::string image) {
  image = trim(image);
  if (image.size() >= 2 && image[0] == '.' &&
      (image[1] == '/' || image[1] == '\\')) {
    image = image.substr(2);
  }
  const bool absolute =
      (!image.empty() && (image[0] == '/' || image[0] == '\\')) ||
      (image.size() >= 3 && std::isalpha(static_cast<unsigned char>(image[0])) &&
       image[1] == ':' && (image[2] == '/' || image[2] == '\\'));
  if (absolute) {
    return image;
  }
  return directory_of(yaml_path) + image;
}

struct MapYaml {
  std::string image{};
  double resolution{0.05};
  mars::common::Point2D origin{};
  double occupied_thresh{0.65};
  int negate{0};
};

MapYaml parse_map_yaml(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open map yaml " + path);
  }
  MapYaml meta;
  bool saw_image = false;
  std::string line;
  while (std::getline(input, line)) {
    const auto comment = line.find('#');
    if (comment != std::string::npos) {
      line = line.substr(0, comment);
    }
    line = trim(line);
    if (line.empty()) {
      continue;
    }
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const std::string key = trim(line.substr(0, colon));
    const std::string value = trim(line.substr(colon + 1));
    if (key == "image") {
      meta.image = value;
      saw_image = true;
    } else if (key == "resolution") {
      meta.resolution = std::stod(value);
    } else if (key == "negate") {
      meta.negate = std::stoi(value);
    } else if (key == "occupied_thresh") {
      meta.occupied_thresh = std::stod(value);
    } else if (key == "origin") {
      const auto open = value.find('[');
      const auto close = value.find(']');
      if (open == std::string::npos || close == std::string::npos || close <= open) {
        throw std::runtime_error("map yaml origin is not a list: " + path);
      }
      std::stringstream list(value.substr(open + 1, close - open - 1));
      std::string item;
      double numbers[3] = {};
      int count = 0;
      while (std::getline(list, item, ',') && count < 3) {
        numbers[count++] = std::stod(trim(item));
      }
      if (count < 2) {
        throw std::runtime_error("map yaml origin needs x and y: " + path);
      }
      meta.origin = {numbers[0], numbers[1]};
    }
  }
  if (!saw_image) {
    throw std::runtime_error("map yaml missing image: " + path);
  }
  return meta;
}

struct PgmImage {
  int width{0};
  int height{0};
  int maxval{0};
  std::vector<unsigned char> pixels{};
};

std::string next_token(const std::string& data, std::size_t& index) {
  const std::size_t size = data.size();
  while (index < size) {
    const unsigned char ch = static_cast<unsigned char>(data[index]);
    if (ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n') {
      ++index;
      continue;
    }
    if (ch == '#') {
      while (index < size && data[index] != '\n' && data[index] != '\r') {
        ++index;
      }
      continue;
    }
    break;
  }
  const std::size_t start = index;
  while (index < size) {
    const unsigned char ch = static_cast<unsigned char>(data[index]);
    if (ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n') {
      break;
    }
    ++index;
  }
  return data.substr(start, index - start);
}

PgmImage read_pgm(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("cannot open map image " + path);
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  const std::string data = buffer.str();
  std::size_t index = 0;
  const std::string magic = next_token(data, index);
  const int width = std::stoi(next_token(data, index));
  const int height = std::stoi(next_token(data, index));
  const int maxval = std::stoi(next_token(data, index));
  if (width <= 0 || height <= 0 || maxval <= 0) {
    throw std::runtime_error("invalid pgm header " + path);
  }
  while (index < data.size()) {
    const unsigned char ch = static_cast<unsigned char>(data[index]);
    if (ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n') {
      ++index;
      continue;
    }
    break;
  }
  const std::size_t count =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  PgmImage image;
  image.width = width;
  image.height = height;
  image.maxval = maxval;
  image.pixels.resize(count);
  if (magic == "P5") {
    if (maxval >= 256) {
      throw std::runtime_error("16-bit pgm is not used by Mode 1 maps");
    }
    if (data.size() < index + count) {
      throw std::runtime_error("pgm is shorter than its header says: " + path);
    }
    for (std::size_t i = 0; i < count; ++i) {
      image.pixels[i] = static_cast<unsigned char>(data[index + i]);
    }
  } else if (magic == "P2") {
    for (std::size_t i = 0; i < count; ++i) {
      image.pixels[i] =
          static_cast<unsigned char>(std::stoi(next_token(data, index)));
    }
  } else {
    throw std::runtime_error("unsupported pgm format " + magic);
  }
  return image;
}

bool occupied_pixel(unsigned char pixel, int maxval, int negate, double thresh) {
  const double value = static_cast<double>(pixel) / static_cast<double>(maxval);
  const double probability = negate == 0 ? 1.0 - value : value;
  return probability > thresh;
}

mars::common::Polygon2D rectangle(double x0, double y0, double x1, double y1) {
  return {{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}};
}

std::vector<mars::common::Polygon2D> obstacles_from_occupied(
    const std::vector<unsigned char>& occupied, int width, int height,
    double resolution, mars::common::Point2D origin) {
  struct Run {
    int col0{0};
    int col1{0};
    int row0{0};
  };
  std::vector<Run> active;
  std::vector<mars::common::Polygon2D> obstacles;
  const auto emit = [&](const Run& run, int row1) {
    const double x0 = origin.x + static_cast<double>(run.col0) * resolution;
    const double x1 = origin.x + static_cast<double>(run.col1 + 1) * resolution;
    const double y0 =
        origin.y + static_cast<double>(height - 1 - row1) * resolution;
    const double y1 =
        origin.y + static_cast<double>(height - run.row0) * resolution;
    obstacles.push_back(rectangle(x0, y0, x1, y1));
  };

  for (int row = 0; row < height; ++row) {
    std::vector<Run> row_runs;
    int col = 0;
    while (col < width) {
      if (occupied[static_cast<std::size_t>(row * width + col)] == 0) {
        ++col;
        continue;
      }
      int end = col;
      while (end + 1 < width &&
             occupied[static_cast<std::size_t>(row * width + end + 1)] != 0) {
        ++end;
      }
      row_runs.push_back({col, end, row});
      col = end + 1;
    }

    std::vector<unsigned char> used(row_runs.size(), 0);
    std::vector<Run> next;
    for (const Run& previous : active) {
      bool extended = false;
      for (std::size_t i = 0; i < row_runs.size(); ++i) {
        if (used[i] != 0) {
          continue;
        }
        if (row_runs[i].col0 == previous.col0 &&
            row_runs[i].col1 == previous.col1) {
          used[i] = 1;
          next.push_back(previous);
          extended = true;
          break;
        }
      }
      if (!extended) {
        emit(previous, row - 1);
      }
    }
    for (std::size_t i = 0; i < row_runs.size(); ++i) {
      if (used[i] == 0) {
        next.push_back(row_runs[i]);
      }
    }
    active = std::move(next);
  }
  for (const Run& previous : active) {
    emit(previous, height - 1);
  }
  return obstacles;
}

Mode1Scenario hard_alley_scenario() {
  Mode1Scenario scenario;
  scenario.id = "hard_alley";
  scenario.occupancy_yaml = "hard_alley_map.yaml";
  scenario.start = {0.35, 0.35};
  scenario.goal = {0.35, 3.40};
  return scenario;
}

Mode1Scenario geogebra_scenario() {
  Mode1Scenario scenario;
  scenario.id = "geogebra";
  scenario.occupancy_yaml = "geogebra_map.yaml";
  scenario.start = {0.35, 0.35};
  scenario.goal = {0.35, 2.90};
  return scenario;
}

}  // namespace

std::vector<std::string> mode1_map_ids() {
  return {"hard_alley", "geogebra"};
}

const Mode1Scenario& mode1_scenario(std::string_view id) {
  static const Mode1Scenario hard_alley = hard_alley_scenario();
  static const Mode1Scenario geogebra = geogebra_scenario();
  if (id == hard_alley.id) {
    return hard_alley;
  }
  if (id == geogebra.id) {
    return geogebra;
  }
  throw std::invalid_argument(
      "unknown Mode 1 map '" + std::string(id) +
      "'; expected hard_alley or geogebra");
}

LoadedMap load_occupancy_map(const std::string& yaml_path) {
  const MapYaml meta = parse_map_yaml(yaml_path);
  const PgmImage image = read_pgm(join_image(yaml_path, meta.image));
  const std::size_t count = static_cast<std::size_t>(image.width) *
                            static_cast<std::size_t>(image.height);
  std::vector<unsigned char> occupied(count, 0);
  for (std::size_t i = 0; i < count; ++i) {
    if (occupied_pixel(image.pixels[i], image.maxval, meta.negate,
                       meta.occupied_thresh)) {
      occupied[i] = 1;
    }
  }

  LoadedMap map;
  map.yaml_path = yaml_path;
  map.resolution = meta.resolution;
  map.origin = meta.origin;
  map.width = image.width;
  map.height = image.height;
  map.obstacles = obstacles_from_occupied(occupied, image.width, image.height,
                                          meta.resolution, meta.origin);
  map.occupancy.resize(count, 0);
  for (std::size_t i = 0; i < count; ++i) {
    if (occupied[i] != 0) {
      map.occupancy[i] = 100;
    }
  }
  return map;
}

}  // namespace mars::demo_simulation

#include <gtest/gtest.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cstdint>
#include <cmath>

struct TelemetryRow {
    uint16_t timestamp_ms;
    int16_t raw_count;
    int16_t dcount_signed;
    int16_t count_unwrapped;
    int16_t theta_unwrapped_e3;
    int16_t omega_e3;
};

struct ReplayRow {
    uint16_t timestamp_ms;
    float theta_logged;
    float theta_replayed;
    float omega_logged;
    float omega_replayed;
};

static std::vector<TelemetryRow> load_csv(const std::string& path) {
    std::vector<TelemetryRow> rows;
    std::ifstream file(path);
    if (!file.is_open()) {
        return rows;
    }

    std::string line;
    std::getline(file, line); // skip header

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string token;
        TelemetryRow r;

        std::getline(ss, token, ','); r.timestamp_ms = (uint16_t)std::stoi(token);
        std::getline(ss, token, ','); r.raw_count = (int16_t)std::stoi(token);
        std::getline(ss, token, ','); r.dcount_signed = (int16_t)std::stoi(token);
        std::getline(ss, token, ','); r.count_unwrapped = (int16_t)std::stoi(token);
        std::getline(ss, token, ','); r.theta_unwrapped_e3 = (int16_t)std::stoi(token);
        std::getline(ss, token, ','); r.omega_e3 = (int16_t)std::stoi(token);

        rows.push_back(r);
    }
    return rows;
}

static std::string make_output_path(const std::string& input_path) {
    auto pos = input_path.find_last_of("/\\");
    std::string dir = (pos != std::string::npos) ? input_path.substr(0, pos + 1) : "";
    std::string name = (pos != std::string::npos) ? input_path.substr(pos + 1) : input_path;
    return dir + "replay_" + name;
}

static void write_replay_csv(const std::string& path, const std::vector<ReplayRow>& results) {
    std::ofstream out(path);
    out << "timestamp_ms,theta_logged,theta_replayed,omega_logged,omega_replayed\n";
    out << std::fixed;
    for (const auto& r : results) {
        out.precision(4);
        out << r.timestamp_ms << ","
            << r.theta_logged << "," << r.theta_replayed << ",";
        out.precision(2);
        out << r.omega_logged << "," << r.omega_replayed << "\n";
    }
}

// Replays the exact firmware wrapping logic against logged data.
// The first row seeds prev_raw_count; replay starts from row 1.
// Returns replay results for CSV output.
static std::vector<ReplayRow> replay_and_verify(const std::vector<TelemetryRow>& rows) {
    std::vector<ReplayRow> results;
    EXPECT_GE(rows.size(), 2u) << "Need at least 2 rows to replay";
    if (rows.size() < 2) return results;

    const float counts_per_rev = 1997.0f;
    const float PI = 3.141592f;
    const float quantization_tol = 1.0f / 1000.0f + 1e-4f;

    int16_t prev_raw_count = rows[0].raw_count;
    int32_t count_unwrapped = rows[0].count_unwrapped;
    uint16_t last_ts = rows[0].timestamp_ms;

    {
        ReplayRow rr;
        rr.timestamp_ms = rows[0].timestamp_ms;
        rr.theta_logged = rows[0].theta_unwrapped_e3 / 1000.0f;
        rr.theta_replayed = count_unwrapped * (2.0f * PI / counts_per_rev);
        rr.omega_logged = rows[0].omega_e3 / 1000.0f;
        rr.omega_replayed = 0.0f;
        results.push_back(rr);
    }

    for (size_t i = 1; i < rows.size(); i++) {
        const TelemetryRow& row = rows[i];

        int16_t raw_count = row.raw_count;
        int16_t dcount_raw = raw_count - prev_raw_count;

        if (dcount_raw > counts_per_rev / 2.0f) {
            dcount_raw -= (int16_t)counts_per_rev;
        } else if (dcount_raw < -counts_per_rev / 2.0f) {
            dcount_raw += (int16_t)counts_per_rev;
        }

        int16_t dcount_signed = -dcount_raw;
        count_unwrapped += dcount_signed;

        float theta_replayed = count_unwrapped * (2.0f * PI / counts_per_rev);
        float theta_logged = row.theta_unwrapped_e3 / 1000.0f;

        uint16_t dt_ms = row.timestamp_ms - last_ts;
        float dt = dt_ms / 1000.0f;
        float omega_replayed = 0.0f;
        if (dt > 0.0f) {
            omega_replayed = (dcount_signed * (2.0f * PI / counts_per_rev)) / dt;
        }
        float omega_logged = row.omega_e3 / 1000.0f;

        EXPECT_EQ(dcount_signed, row.dcount_signed)
            << "Row " << i << ": dcount_signed mismatch";
        EXPECT_EQ((int16_t)count_unwrapped, row.count_unwrapped)
            << "Row " << i << ": count_unwrapped mismatch";
        EXPECT_NEAR(theta_replayed, theta_logged, quantization_tol)
            << "Row " << i << ": theta mismatch (float)";
        EXPECT_NEAR(omega_replayed, omega_logged, quantization_tol)
            << "Row " << i << ": omega mismatch (float)";

        ReplayRow rr;
        rr.timestamp_ms = row.timestamp_ms;
        rr.theta_logged = theta_logged;
        rr.theta_replayed = theta_replayed;
        rr.omega_logged = omega_logged;
        rr.omega_replayed = omega_replayed;
        results.push_back(rr);

        prev_raw_count = raw_count;
        last_ts = row.timestamp_ms;
    }
    return results;
}

static std::vector<std::string> csv_paths;

static std::string sanitize_name(const std::string& path) {
    std::string name = path;
    auto pos = name.find_last_of("/\\");
    if (pos != std::string::npos) name = name.substr(pos + 1);
    for (auto& c : name) {
        if (!isalnum(c)) c = '_';
    }
    return name;
}

class CsvReplayTest : public ::testing::Test {
public:
    std::string csv_path;
    void TestBody() override {
        std::vector<TelemetryRow> rows = load_csv(csv_path);
        ASSERT_FALSE(rows.empty()) << "Failed to load CSV: " << csv_path;
        std::vector<ReplayRow> results = replay_and_verify(rows);
        std::string out_path = make_output_path(csv_path);
        write_replay_csv(out_path, results);
        std::cout << "Replay CSV written to: " << out_path << std::endl;
    }
};

static void register_csv_tests() {
    for (const auto& path : csv_paths) {
        ::testing::RegisterTest(
            "CsvFiles", ("ReplayMatchesFirmware/" + sanitize_name(path)).c_str(),
            nullptr, sanitize_name(path).c_str(),
            __FILE__, __LINE__,
            [path]() -> CsvReplayTest* {
                auto* t = new CsvReplayTest();
                t->csv_path = path;
                return t;
            });
    }
}

// Synthetic test: steady rotation with no wrapping
TEST(EncoderUnwrapSynthetic, SteadyNoWrap) {
    const float counts_per_rev = 1997.0f;
    const float PI = 3.141592f;
    std::vector<TelemetryRow> rows;

    int16_t raw = 0;
    int32_t accum = 0;
    for (int i = 0; i < 50; i++) {
        TelemetryRow r;
        r.timestamp_ms = i;
        r.raw_count = raw;

        int16_t dcount_raw = (i == 0) ? 0 : 10;
        int16_t dsigned = -dcount_raw;
        if (i > 0) accum += dsigned;

        r.dcount_signed = dsigned;
        r.count_unwrapped = (int16_t)accum;
        r.theta_unwrapped_e3 = (int16_t)(accum * (2.0f * PI / counts_per_rev) * 1000.0f);

        float dt = (i == 0) ? 0.0f : 0.001f;
        float omega = (dt > 0.0f) ? (dsigned * (2.0f * PI / counts_per_rev)) / dt : 0.0f;
        r.omega_e3 = (int16_t)(omega * 1000.0f);

        rows.push_back(r);
        raw += 10;
    }
    replay_and_verify(rows);
}

// Synthetic test: forward wrap (counter rolls over from high to low)
TEST(EncoderUnwrapSynthetic, ForwardWrap) {
    const float counts_per_rev = 1997.0f;
    const float PI = 3.141592f;
    std::vector<TelemetryRow> rows;

    int16_t raw_sequence[] = {1990, 1995, 3, 8, 13};
    int32_t accum = 0;
    int16_t prev = raw_sequence[0];

    for (int i = 0; i < 5; i++) {
        TelemetryRow r;
        r.timestamp_ms = i;
        r.raw_count = raw_sequence[i];

        int16_t dcount_raw = 0;
        if (i > 0) {
            dcount_raw = raw_sequence[i] - prev;
            if (dcount_raw > counts_per_rev / 2.0f)
                dcount_raw -= (int16_t)counts_per_rev;
            else if (dcount_raw < -counts_per_rev / 2.0f)
                dcount_raw += (int16_t)counts_per_rev;
        }
        int16_t dsigned = -dcount_raw;
        if (i > 0) accum += dsigned;

        r.dcount_signed = dsigned;
        r.count_unwrapped = (int16_t)accum;
        r.theta_unwrapped_e3 = (int16_t)(accum * (2.0f * PI / counts_per_rev) * 1000.0f);

        float dt = (i == 0) ? 0.0f : 0.001f;
        float omega = (dt > 0.0f) ? (dsigned * (2.0f * PI / counts_per_rev)) / dt : 0.0f;
        r.omega_e3 = (int16_t)(omega * 1000.0f);

        rows.push_back(r);
        prev = raw_sequence[i];
    }
    replay_and_verify(rows);
}

// Synthetic test: reverse wrap (counter rolls under from low to high)
TEST(EncoderUnwrapSynthetic, ReverseWrap) {
    const float counts_per_rev = 1997.0f;
    const float PI = 3.141592f;
    std::vector<TelemetryRow> rows;

    int16_t raw_sequence[] = {5, 0, 1992, 1987, 1982};
    int32_t accum = 0;
    int16_t prev = raw_sequence[0];

    for (int i = 0; i < 5; i++) {
        TelemetryRow r;
        r.timestamp_ms = i;
        r.raw_count = raw_sequence[i];

        int16_t dcount_raw = 0;
        if (i > 0) {
            dcount_raw = raw_sequence[i] - prev;
            if (dcount_raw > counts_per_rev / 2.0f)
                dcount_raw -= (int16_t)counts_per_rev;
            else if (dcount_raw < -counts_per_rev / 2.0f)
                dcount_raw += (int16_t)counts_per_rev;
        }
        int16_t dsigned = -dcount_raw;
        if (i > 0) accum += dsigned;

        r.dcount_signed = dsigned;
        r.count_unwrapped = (int16_t)accum;
        r.theta_unwrapped_e3 = (int16_t)(accum * (2.0f * PI / counts_per_rev) * 1000.0f);

        float dt = (i == 0) ? 0.0f : 0.001f;
        float omega = (dt > 0.0f) ? (dsigned * (2.0f * PI / counts_per_rev)) / dt : 0.0f;
        r.omega_e3 = (int16_t)(omega * 1000.0f);

        rows.push_back(r);
        prev = raw_sequence[i];
    }
    replay_and_verify(rows);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--csv" && i + 1 < argc) {
            csv_paths.push_back(argv[++i]);
        } else if (arg.size() > 4 && arg.substr(arg.size() - 4) == ".csv") {
            csv_paths.push_back(arg);
        }
    }

    if (csv_paths.empty()) {
        std::cout << "No CSV files provided. Usage: ./test_encoder_unwrap data.csv [data2.csv ...]\n";
        std::cout << "Running synthetic tests only.\n";
    } else {
        register_csv_tests();
    }

    return RUN_ALL_TESTS();
}

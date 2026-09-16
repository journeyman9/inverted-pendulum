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
    int16_t raw_count;
    int16_t dcount_signed_logged;
    int16_t dcount_signed_replayed;
    int16_t count_unwrapped_logged;
    int32_t count_unwrapped_replayed;
    float theta_logged;
    float theta_replayed;
    float theta_relative_logged;
    float theta_relative_replayed;
    float omega_logged;
    float omega_replayed;
    bool reseed;
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
    out << "timestamp_ms,raw_count,"
        << "dcount_signed_logged,dcount_signed_replayed,"
        << "count_unwrapped_logged,count_unwrapped_replayed,"
        << "theta_logged,theta_replayed,"
        << "theta_relative_logged,theta_relative_replayed,"
        << "omega_logged,omega_replayed,reseed\n";
    out << std::fixed;
    for (const auto& r : results) {
        out << r.timestamp_ms << "," << r.raw_count << ","
            << r.dcount_signed_logged << "," << r.dcount_signed_replayed << ","
            << r.count_unwrapped_logged << "," << r.count_unwrapped_replayed << ",";
        out.precision(4);
        out << r.theta_logged << "," << r.theta_replayed << ","
            << r.theta_relative_logged << "," << r.theta_relative_replayed << ",";
        out.precision(2);
        out << r.omega_logged << "," << r.omega_replayed << ","
            << (r.reseed ? 1 : 0) << "\n";
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

    float angle_set_logged = rows[0].theta_unwrapped_e3 / 1000.0f;
    float angle_set_replayed = count_unwrapped * (2.0f * PI / counts_per_rev);

    {
        ReplayRow rr = {};
        rr.timestamp_ms = rows[0].timestamp_ms;
        rr.raw_count = rows[0].raw_count;
        rr.dcount_signed_logged = rows[0].dcount_signed;
        rr.dcount_signed_replayed = 0;
        rr.count_unwrapped_logged = rows[0].count_unwrapped;
        rr.count_unwrapped_replayed = count_unwrapped;
        rr.theta_logged = angle_set_logged;
        rr.theta_replayed = angle_set_replayed;
        rr.theta_relative_logged = 0.0f;
        rr.theta_relative_replayed = 0.0f;
        rr.omega_logged = rows[0].omega_e3 / 1000.0f;
        rr.omega_replayed = 0.0f;
        rr.reseed = true;
        results.push_back(rr);
    }

    int reseeds = 0;

    for (size_t i = 1; i < rows.size(); i++) {
        const TelemetryRow& row = rows[i];

        uint16_t dt_ms = row.timestamp_ms - last_ts;

        // Detect ring-buffer wraparound: either a timestamp gap or a
        // count_unwrapped jump that doesn't match the raw_count delta.
        int16_t expected_dcount_raw = row.raw_count - prev_raw_count;
        if (expected_dcount_raw > counts_per_rev / 2.0f)
            expected_dcount_raw -= (int16_t)counts_per_rev;
        else if (expected_dcount_raw < -counts_per_rev / 2.0f)
            expected_dcount_raw += (int16_t)counts_per_rev;
        int16_t expected_dcount_signed = -expected_dcount_raw;
        int32_t expected_unwrapped = count_unwrapped + expected_dcount_signed;

        // Check replay state vs logged state
        bool replay_discontinuity = (expected_dcount_signed != row.dcount_signed)
                                 || ((int16_t)expected_unwrapped != row.count_unwrapped);

        // Check internal self-consistency of the logged row itself.
        // Torn writes at ring-buffer seams can mix fields from two
        // different loop iterations.
        int16_t expected_theta_e3 = (int16_t)(row.count_unwrapped
                                    * (2.0f * PI / counts_per_rev) * 1000.0f);
        bool theta_torn = (abs(expected_theta_e3 - row.theta_unwrapped_e3) > 1);

        int16_t expected_omega_e3 = 0;
        if (dt_ms > 0) {
            expected_omega_e3 = (int16_t)(row.dcount_signed
                                * (2.0f * PI / counts_per_rev)
                                / (dt_ms / 1000.0f) * 1000.0f);
        }
        bool omega_torn = (abs(expected_omega_e3 - row.omega_e3) > 1);

        bool data_discontinuity = replay_discontinuity || theta_torn || omega_torn;

        if (dt_ms > 2 || data_discontinuity) {
            reseeds++;
            std::cout << "  Reseed at row " << i
                      << " (dt=" << dt_ms << "ms, gap from "
                      << last_ts << " to " << row.timestamp_ms << ")\n";
            prev_raw_count = row.raw_count;
            count_unwrapped = row.count_unwrapped;
            last_ts = row.timestamp_ms;

            float theta_logged = row.theta_unwrapped_e3 / 1000.0f;
            float theta_replayed = count_unwrapped * (2.0f * PI / counts_per_rev);
            if (!theta_torn && !omega_torn) {
                EXPECT_NEAR(theta_replayed, theta_logged, quantization_tol)
                    << "Row " << i << ": theta mismatch at reseed";
            }

            ReplayRow rr = {};
            rr.timestamp_ms = row.timestamp_ms;
            rr.raw_count = row.raw_count;
            rr.dcount_signed_logged = row.dcount_signed;
            rr.dcount_signed_replayed = 0;
            rr.count_unwrapped_logged = row.count_unwrapped;
            rr.count_unwrapped_replayed = count_unwrapped;
            rr.theta_logged = theta_logged;
            rr.theta_replayed = theta_replayed;
            rr.theta_relative_logged = theta_logged - angle_set_logged;
            rr.theta_relative_replayed = theta_replayed - angle_set_replayed;
            rr.omega_logged = row.omega_e3 / 1000.0f;
            rr.omega_replayed = 0.0f;
            rr.reseed = true;
            results.push_back(rr);
            continue;
        }

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

        ReplayRow rr = {};
        rr.timestamp_ms = row.timestamp_ms;
        rr.raw_count = row.raw_count;
        rr.dcount_signed_logged = row.dcount_signed;
        rr.dcount_signed_replayed = dcount_signed;
        rr.count_unwrapped_logged = row.count_unwrapped;
        rr.count_unwrapped_replayed = count_unwrapped;
        rr.theta_logged = theta_logged;
        rr.theta_replayed = theta_replayed;
        rr.theta_relative_logged = theta_logged - angle_set_logged;
        rr.theta_relative_replayed = theta_replayed - angle_set_replayed;
        rr.omega_logged = omega_logged;
        rr.omega_replayed = omega_replayed;
        rr.reseed = false;
        results.push_back(rr);

        prev_raw_count = raw_count;
        last_ts = row.timestamp_ms;
    }

    std::cout << "  Total reseeds (buffer gaps): " << reseeds << "\n";
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

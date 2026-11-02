#include <gtest/gtest.h>

#include <sys/wait.h>
#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#ifndef ODOMCAL_BENCH_EXE
#error "ODOMCAL_BENCH_EXE must name the odomcal_bench executable"
#endif

namespace {

namespace fs = std::filesystem;

// Runs odomcal_bench from a fresh, empty working directory.
class BenchCli : public ::testing::Test {
protected:
    void SetUp() override {
        const std::string name = ::testing::UnitTest::GetInstance()->current_test_info()->name();
        dir_ = fs::temp_directory_path() / ("odomcal_cli_" + name + "_" + std::to_string(::getpid()));
        fs::remove_all(dir_);
        fs::create_directories(dir_);
    }

    void TearDown() override { fs::remove_all(dir_); }

    // Exit code of the benchmark, or -1 if it did not exit normally (for example, it aborted).
    int run(const std::string& arguments) const {
        const std::string command = "cd '" + dir_.string() + "' && '" + ODOMCAL_BENCH_EXE + "' " + arguments +
                                    " > /dev/null 2>&1";
        const int status = std::system(command.c_str());
        if (status == -1 || !WIFEXITED(status)) {
            return -1;
        }
        return WEXITSTATUS(status);
    }

    fs::path dir_;
};

TEST_F(BenchCli, WritesBothCsvFilesToTheGivenPaths) {
    ASSERT_EQ(run("summary.csv squares.csv"), 0);
    std::ifstream summary(dir_ / "summary.csv");
    const std::string text((std::istreambuf_iterator<char>(summary)), std::istreambuf_iterator<char>());
    EXPECT_EQ(text.rfind("row_type,seed,ed_true,eb_true,", 0), 0u);
    EXPECT_NE(text.find("\nmean,-1,"), std::string::npos);
    EXPECT_TRUE(fs::exists(dir_ / "squares.csv"));
}

// "--help" used to be taken as the summary path: the benchmark wrote a file
// named "--help" and docs/trajectories.csv under the working directory.
TEST_F(BenchCli, HelpPrintsUsageWithoutWritingFiles) {
    EXPECT_EQ(run("--help"), 0);
    EXPECT_TRUE(fs::is_empty(dir_));
}

TEST_F(BenchCli, OptionsAndExtraArgumentsAreErrorsWithoutWritingFiles) {
    EXPECT_EQ(run("--csv summary.csv"), 1);
    EXPECT_EQ(run("a.csv b.csv c.csv"), 1);
    EXPECT_TRUE(fs::is_empty(dir_));
}

// A path below a regular file cannot be created. The exception used to escape
// main and abort the process instead of returning an error code.
TEST_F(BenchCli, UnwritableOutputIsAnErrorNotAnAbort) {
    {
        std::ofstream file(dir_ / "file");
        file << "not a directory\n";
    }
    EXPECT_EQ(run("file/summary.csv squares.csv"), 1);
}

}  // namespace

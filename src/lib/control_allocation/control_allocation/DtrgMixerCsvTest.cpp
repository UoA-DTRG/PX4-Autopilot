/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file DtrgMixerCsvTest.cpp
 *
 * Parser of the DTRG CSV mixer (DTRG_MIXER_CSV), which replaces the
 * pseudo-inverse of the effectiveness matrix with a matrix read from a file:
 * one row per actuator, one column per axis (roll, pitch, yaw, x, y, z).
 */

#include <gtest/gtest.h>

#include <stdio.h>
#include <string>

#include <ControlAllocationPseudoInverse.hpp>
#include <parameters/param.h>

namespace
{

using Mixer = matrix::Matrix<float, ControlAllocation::NUM_ACTUATORS, ControlAllocation::NUM_AXES>;

constexpr float kUntouched = 99.f;

class DtrgMixerCsv : public ::testing::Test
{
protected:
	void SetUp() override
	{
		const ::testing::TestInfo *info = ::testing::UnitTest::GetInstance()->current_test_info();
		_path = ::testing::TempDir() + "dtrg_mixer_" + info->name() + ".csv";

		for (int r = 0; r < ControlAllocation::NUM_ACTUATORS; r++) {
			for (int c = 0; c < ControlAllocation::NUM_AXES; c++) {
				_mixer(r, c) = kUntouched;
			}
		}
	}

	void TearDown() override
	{
		remove(_path.c_str());
	}

	void writeFile(const std::string &content)
	{
		FILE *f = fopen(_path.c_str(), "wb");
		ASSERT_NE(f, nullptr);
		fwrite(content.data(), 1, content.size(), f);
		fclose(f);
	}

	bool read()
	{
		return ControlAllocationPseudoInverse::readMixerFromCSV(_path.c_str(), _mixer);
	}

	/// "r.c" style value that makes misplaced cells obvious in a failure message
	static float cell(int r, int c) { return r + 0.1f * (c + 1); }

	static std::string rows(int num_rows, const char *eol = "\n")
	{
		std::string s;

		for (int r = 0; r < num_rows; r++) {
			for (int c = 0; c < ControlAllocation::NUM_AXES; c++) {
				char buf[32];
				snprintf(buf, sizeof(buf), "%s%.1f", c ? "," : "", (double)cell(r, c));
				s += buf;
			}

			s += eol;
		}

		return s;
	}

	void expectRows(int num_rows)
	{
		for (int r = 0; r < num_rows; r++) {
			for (int c = 0; c < ControlAllocation::NUM_AXES; c++) {
				EXPECT_FLOAT_EQ(_mixer(r, c), cell(r, c)) << "row " << r << " col " << c;
			}
		}
	}

	std::string _path;
	Mixer _mixer;
};

} // namespace

TEST_F(DtrgMixerCsv, MissingFileIsRejectedAndMixerUntouched)
{
	EXPECT_FALSE(ControlAllocationPseudoInverse::readMixerFromCSV("/nonexistent/dtrg/mixer.csv", _mixer));
	EXPECT_FLOAT_EQ(_mixer(0, 0), kUntouched);
	EXPECT_FLOAT_EQ(_mixer(ControlAllocation::NUM_ACTUATORS - 1, ControlAllocation::NUM_AXES - 1), kUntouched);
}

TEST_F(DtrgMixerCsv, ReadsOctoMatrix)
{
	writeFile(rows(8));
	ASSERT_TRUE(read());
	expectRows(8);
}

TEST_F(DtrgMixerCsv, RowsBeyondTheFileAreUntouched)
{
	writeFile(rows(8));
	ASSERT_TRUE(read());
	EXPECT_FLOAT_EQ(_mixer(8, 0), kUntouched);
	EXPECT_FLOAT_EQ(_mixer(ControlAllocation::NUM_ACTUATORS - 1, 0), kUntouched);
}

TEST_F(DtrgMixerCsv, Utf8BomIsSkipped)
{
	writeFile("\xEF\xBB\xBF" + rows(8));
	ASSERT_TRUE(read());
	expectRows(8);
}

TEST_F(DtrgMixerCsv, CrlfLineEndings)
{
	writeFile(rows(8, "\r\n"));
	ASSERT_TRUE(read());
	expectRows(8);
}

TEST_F(DtrgMixerCsv, NoTrailingNewline)
{
	std::string content = rows(8);
	content.pop_back();
	writeFile(content);
	ASSERT_TRUE(read());
	expectRows(8);
}

TEST_F(DtrgMixerCsv, BlankLinesAreSkipped)
{
	const std::string r = rows(2);
	const size_t split = r.find('\n') + 1;
	writeFile("\n" + r.substr(0, split) + "\n\n" + r.substr(split));
	ASSERT_TRUE(read());
	expectRows(2);
	EXPECT_FLOAT_EQ(_mixer(2, 0), kUntouched);
}

TEST_F(DtrgMixerCsv, NegativeAndScientificValues)
{
	writeFile("-0.5,1e-1,-2.5E-2,0,+1,-1\n");
	ASSERT_TRUE(read());
	EXPECT_FLOAT_EQ(_mixer(0, 0), -0.5f);
	EXPECT_FLOAT_EQ(_mixer(0, 1), 0.1f);
	EXPECT_FLOAT_EQ(_mixer(0, 2), -0.025f);
	EXPECT_FLOAT_EQ(_mixer(0, 3), 0.f);
	EXPECT_FLOAT_EQ(_mixer(0, 4), 1.f);
	EXPECT_FLOAT_EQ(_mixer(0, 5), -1.f);
}

TEST_F(DtrgMixerCsv, ExtraColumnsAreIgnored)
{
	writeFile("1,2,3,4,5,6,7,8\n");
	ASSERT_TRUE(read());
	EXPECT_FLOAT_EQ(_mixer(0, 5), 6.f);
	EXPECT_FLOAT_EQ(_mixer(1, 0), kUntouched); // the extra cells did not spill into the next row
}

TEST_F(DtrgMixerCsv, ExtraRowsAreIgnored)
{
	writeFile(rows(ControlAllocation::NUM_ACTUATORS + 4));
	ASSERT_TRUE(read());
	expectRows(ControlAllocation::NUM_ACTUATORS);
}

// A full precision export (e.g. MATLAB writematrix, -0.35355339059327373) is
// ~20 characters per cell, so a 6 column row is longer than 100 characters. It
// must be read as one row, not split in two at a fixed size line buffer.
TEST_F(DtrgMixerCsv, FullPrecisionRowsAreNotSplit)
{
	const char *row = "-0.35355339059327373,0.35355339059327373,-0.12500000000000000,"
			  "0.00000000000000000,0.00000000000000000,-0.12500000000000000\n";
	writeFile(std::string(row) + row);
	ASSERT_TRUE(read());
	EXPECT_FLOAT_EQ(_mixer(0, 5), -0.125f);
	EXPECT_FLOAT_EQ(_mixer(1, 0), -0.35355339f);
	EXPECT_FLOAT_EQ(_mixer(2, 0), kUntouched);
}

// A cell too long to be a number is rejected rather than cut short.
TEST_F(DtrgMixerCsv, OverlongCellIsRejected)
{
	writeFile("1,2,3,4,5,0.000000000000000000000000000000000000001\n");
	EXPECT_FALSE(read());
	EXPECT_FLOAT_EQ(_mixer(0, 0), kUntouched);
}

// Spreadsheets write an empty cell as two consecutive commas. It must not shift
// the rest of the row one column to the left (as strtok() used to).
TEST_F(DtrgMixerCsv, EmptyCellKeepsColumnPosition)
{
	writeFile("1,,3,4,5,6\n");
	ASSERT_TRUE(read());
	EXPECT_FLOAT_EQ(_mixer(0, 2), 3.f);
	EXPECT_FLOAT_EQ(_mixer(0, 5), 6.f);
}

// An empty file would leave the allocator without a mixer.
TEST_F(DtrgMixerCsv, EmptyFileIsRejected)
{
	writeFile("");
	EXPECT_FALSE(read());
}

// A row with fewer than 6 cells would leave the missing axes at whatever the
// matrix held before.
TEST_F(DtrgMixerCsv, ShortRowIsRejected)
{
	writeFile("1,2,3\n");
	EXPECT_FALSE(read());
}

// A blank line with a Windows line ending must not be read as a row of zeros.
TEST_F(DtrgMixerCsv, BlankCrlfLineIsSkipped)
{
	writeFile("\r\n" + rows(1, "\r\n"));
	ASSERT_TRUE(read());
	expectRows(1);
}

// Why a file was rejected, and where, is reported (for the commander arming check).
using Status = ControlAllocation::CsvMixerStatus;

TEST_F(DtrgMixerCsv, ResultOfValidFile)
{
	writeFile(rows(8));
	ControlAllocation::CsvMixerResult result;
	ASSERT_TRUE(ControlAllocationPseudoInverse::readMixerFromCSV(_path.c_str(), _mixer, &result));
	EXPECT_EQ(result.status, Status::LOADED);
	EXPECT_EQ(result.num_rows, 8);
	EXPECT_EQ(result.line, 0);
}

TEST_F(DtrgMixerCsv, ResultOfMissingFile)
{
	ControlAllocation::CsvMixerResult result;
	EXPECT_FALSE(ControlAllocationPseudoInverse::readMixerFromCSV("/nonexistent/dtrg/mixer.csv", _mixer, &result));
	EXPECT_EQ(result.status, Status::FILE_NOT_FOUND);
}

TEST_F(DtrgMixerCsv, ResultOfEmptyFile)
{
	writeFile("\n\n");
	ControlAllocation::CsvMixerResult result;
	EXPECT_FALSE(ControlAllocationPseudoInverse::readMixerFromCSV(_path.c_str(), _mixer, &result));
	EXPECT_EQ(result.status, Status::EMPTY);
}

// The line counts blank lines, so that it matches what an editor shows
TEST_F(DtrgMixerCsv, ShortRowReportsItsLine)
{
	writeFile("\n" + rows(1) + "1,2,3\n" + rows(1));
	ControlAllocation::CsvMixerResult result;
	EXPECT_FALSE(ControlAllocationPseudoInverse::readMixerFromCSV(_path.c_str(), _mixer, &result));
	EXPECT_EQ(result.status, Status::SHORT_ROW);
	EXPECT_EQ(result.line, 3);
	EXPECT_FLOAT_EQ(_mixer(0, 0), kUntouched);
}

// A header row would otherwise read as a row of zeros
TEST_F(DtrgMixerCsv, HeaderRowIsRejected)
{
	writeFile("roll,pitch,yaw,x,y,z\n" + rows(8));
	ControlAllocation::CsvMixerResult result;
	EXPECT_FALSE(ControlAllocationPseudoInverse::readMixerFromCSV(_path.c_str(), _mixer, &result));
	EXPECT_EQ(result.status, Status::INVALID_VALUE);
	EXPECT_EQ(result.line, 1);
	EXPECT_FLOAT_EQ(_mixer(0, 0), kUntouched);
}

TEST_F(DtrgMixerCsv, NonNumericCellReportsItsLine)
{
	writeFile(rows(1, "\r\n") + "\r\n" + "1,2,3x,4,5,6\r\n");
	ControlAllocation::CsvMixerResult result;
	EXPECT_FALSE(ControlAllocationPseudoInverse::readMixerFromCSV(_path.c_str(), _mixer, &result));
	EXPECT_EQ(result.status, Status::INVALID_VALUE);
	EXPECT_EQ(result.line, 3);
}

TEST_F(DtrgMixerCsv, NanAndInfAreRejected)
{
	writeFile("nan,0,0,0,0,-1\n");
	EXPECT_FALSE(read());
	writeFile("0,0,0,0,0,inf\n");
	EXPECT_FALSE(read());
	EXPECT_FLOAT_EQ(_mixer(0, 0), kUntouched);
}

TEST_F(DtrgMixerCsv, OverlongCellIsAnInvalidValue)
{
	writeFile("1,2,3,4,5,0.000000000000000000000000000000000000001\n");
	ControlAllocation::CsvMixerResult result;
	EXPECT_FALSE(ControlAllocationPseudoInverse::readMixerFromCSV(_path.c_str(), _mixer, &result));
	EXPECT_EQ(result.status, Status::INVALID_VALUE);
	EXPECT_EQ(result.line, 1);
}

// Loading in the allocator ----------------------------------------------------

namespace
{
class TestAllocation : public ControlAllocationPseudoInverse
{
public:
	using ControlAllocationPseudoInverse::updateParams;
	void setCsvMixerPath(const char *path) { _csv_mixer_path = path; }
};

class DtrgMixerCsvLoad : public ::testing::Test
{
protected:
	void SetUp() override
	{
		// Disable autosaving parameters to avoid busy loop in param_set()
		param_control_autosave(false);

		// Normalization would rescale the file values
		int32_t normalization = 0;
		ASSERT_EQ(param_set(param_find("DTRG_MIXER_NORM"), &normalization), PX4_OK);
		setCsvMixer(true);

		const ::testing::TestInfo *info = ::testing::UnitTest::GetInstance()->current_test_info();
		_path = ::testing::TempDir() + "dtrg_mixer_load_" + info->name() + ".csv";
		_allocation.setCsvMixerPath(_path.c_str());
	}

	void TearDown() override
	{
		remove(_path.c_str());
		param_reset(param_find("DTRG_MIXER_CSV"));
		param_reset(param_find("DTRG_MIXER_NORM"));
	}

	void setCsvMixer(bool enabled)
	{
		int32_t value = enabled;
		ASSERT_EQ(param_set(param_find("DTRG_MIXER_CSV"), &value), PX4_OK);
		_allocation.updateParams();
	}

	void writeFile(const std::string &content)
	{
		FILE *f = fopen(_path.c_str(), "wb");
		ASSERT_NE(f, nullptr);
		fwrite(content.data(), 1, content.size(), f);
		fclose(f);
	}

	/// Set a quad X geometry, which makes the allocator (re)load the mixer, and return the mixer
	Mixer load(int num_actuators = 4)
	{
		matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> effectiveness;
		const float roll[4] {-1.f, 1.f, 1.f, -1.f};
		const float pitch[4] {1.f, -1.f, 1.f, -1.f};
		const float yaw[4] {1.f, 1.f, -1.f, -1.f};

		for (int i = 0; i < num_actuators; i++) {
			effectiveness(0, i) = roll[i % 4];
			effectiveness(1, i) = pitch[i % 4];
			effectiveness(2, i) = yaw[i % 4];
			effectiveness(5, i) = -1.f;
		}

		_allocation.setEffectivenessMatrix(effectiveness, ControlAllocation::ActuatorVector{},
						   ControlAllocation::ActuatorVector{}, num_actuators, true);
		Mixer mixer;
		EXPECT_TRUE(_allocation.getMixer(mixer));
		return mixer;
	}

	static bool isZero(const Mixer &mixer) { return mixer.abs().max() <= 0.f; }

	static constexpr const char *kQuad =
		"-0.5,0.5,0.25,0,0,-0.25\n"
		"0.5,-0.5,0.25,0,0,-0.25\n"
		"0.5,0.5,-0.25,0,0,-0.25\n"
		"-0.5,-0.5,-0.25,0,0,-0.25\n";

	std::string _path;
	TestAllocation _allocation;
};

} // namespace

TEST_F(DtrgMixerCsvLoad, ValidFileIsUsed)
{
	writeFile(kQuad);
	const Mixer mixer = load();
	EXPECT_EQ(_allocation.getCsvMixerResult().status, Status::LOADED);
	EXPECT_FLOAT_EQ(mixer(0, 0), -0.5f);
	EXPECT_FLOAT_EQ(mixer(3, 2), -0.25f);
	EXPECT_FLOAT_EQ(mixer(2, 5), -0.25f);
}

TEST_F(DtrgMixerCsvLoad, DisabledReportsDisabled)
{
	setCsvMixer(false);
	const Mixer mixer = load();
	EXPECT_EQ(_allocation.getCsvMixerResult().status, Status::DISABLED);
	EXPECT_FALSE(isZero(mixer)); // the pseudo-inverse
}

// No usable file: the allocator uses an empty mixer rather than keeping the
// previous (pseudo-inverse) one, and commander refuses to arm
TEST_F(DtrgMixerCsvLoad, MissingFileGivesEmptyMixer)
{
	setCsvMixer(false);
	ASSERT_FALSE(isZero(load()));

	setCsvMixer(true);
	EXPECT_TRUE(isZero(load()));
	EXPECT_EQ(_allocation.getCsvMixerResult().status, Status::FILE_NOT_FOUND);
}

TEST_F(DtrgMixerCsvLoad, InvalidFileGivesEmptyMixer)
{
	writeFile("roll,pitch,yaw,x,y,z\n" + std::string(kQuad));
	EXPECT_TRUE(isZero(load()));
	EXPECT_EQ(_allocation.getCsvMixerResult().status, Status::INVALID_VALUE);
	EXPECT_EQ(_allocation.getCsvMixerResult().line, 1);
}

TEST_F(DtrgMixerCsvLoad, FewerRowsThanActuatorsIsRejected)
{
	writeFile(kQuad);
	EXPECT_TRUE(isZero(load(8)));
	EXPECT_EQ(_allocation.getCsvMixerResult().status, Status::ROW_COUNT_MISMATCH);
	EXPECT_EQ(_allocation.getCsvMixerResult().num_rows, 4);
	EXPECT_EQ(_allocation.numConfiguredActuators(), 8);
}

TEST_F(DtrgMixerCsvLoad, MoreRowsThanActuatorsIsRejected)
{
	writeFile(std::string(kQuad) + kQuad);
	EXPECT_TRUE(isZero(load(4)));
	EXPECT_EQ(_allocation.getCsvMixerResult().status, Status::ROW_COUNT_MISMATCH);
	EXPECT_EQ(_allocation.getCsvMixerResult().num_rows, 8);
}

TEST_F(DtrgMixerCsvLoad, AllZeroFileIsRejected)
{
	writeFile("0,0,0,0,0,0\n0,0,0,0,0,0\n0,0,0,0,0,0\n0,0,0,0,0,0\n");
	EXPECT_TRUE(isZero(load()));
	EXPECT_EQ(_allocation.getCsvMixerResult().status, Status::ALL_ZERO);
}

// The file is re-read whenever the effectiveness is updated, e.g. on a parameter
// change in flight. A failed re-read reports the error (so the vehicle cannot be
// armed again) but must not cut the motors.
TEST_F(DtrgMixerCsvLoad, FailedReloadKeepsLastValidMixer)
{
	writeFile(kQuad);
	const Mixer loaded = load();
	ASSERT_EQ(_allocation.getCsvMixerResult().status, Status::LOADED);

	remove(_path.c_str());
	const Mixer reloaded = load();
	EXPECT_EQ(_allocation.getCsvMixerResult().status, Status::FILE_NOT_FOUND);
	EXPECT_TRUE(isEqual(reloaded, loaded));

	// and a fixed file is picked up again
	writeFile(std::string(kQuad).replace(0, 4, "-0.4"));
	EXPECT_FLOAT_EQ(load()(0, 0), -0.4f);
	EXPECT_EQ(_allocation.getCsvMixerResult().status, Status::LOADED);
}

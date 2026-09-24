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
 *
 * The DISABLED_ tests describe known gaps in the parser. They are compiled
 * but not run; enable one once the parser is fixed. Run them anyway with
 * --gtest_also_run_disabled_tests.
 */

#include <gtest/gtest.h>

#include <stdio.h>
#include <string>

#include <ControlAllocationPseudoInverse.hpp>

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

// Known gaps ------------------------------------------------------------------

// A full precision export (e.g. MATLAB writematrix, -0.35355339059327373) is
// ~20 characters per cell, so a 6 column row is longer than the 100 byte line
// buffer. The rest of the line is read as the next actuator's row, shifting
// every following row.
TEST_F(DtrgMixerCsv, DISABLED_FullPrecisionRowsAreNotSplit)
{
	const char *row = "-0.35355339059327373,0.35355339059327373,-0.12500000000000000,"
			  "0.00000000000000000,0.00000000000000000,-0.12500000000000000\n";
	writeFile(std::string(row) + row);
	ASSERT_TRUE(read());
	EXPECT_FLOAT_EQ(_mixer(0, 5), -0.125f);
	EXPECT_FLOAT_EQ(_mixer(1, 0), -0.35355339f);
	EXPECT_FLOAT_EQ(_mixer(2, 0), kUntouched);
}

// strtok() merges consecutive delimiters, so an empty cell shifts the rest of
// the row one column to the left. Spreadsheets write empty cells this way.
TEST_F(DtrgMixerCsv, DISABLED_EmptyCellKeepsColumnPosition)
{
	writeFile("1,,3,4,5,6\n");
	ASSERT_TRUE(read());
	EXPECT_FLOAT_EQ(_mixer(0, 2), 3.f);
	EXPECT_FLOAT_EQ(_mixer(0, 5), 6.f);
}

// An empty file is accepted, which leaves the allocator without a mixer.
TEST_F(DtrgMixerCsv, DISABLED_EmptyFileIsRejected)
{
	writeFile("");
	EXPECT_FALSE(read());
}

// A row with fewer than 6 cells is accepted and the missing axes keep
// whatever the matrix held before.
TEST_F(DtrgMixerCsv, DISABLED_ShortRowIsRejected)
{
	writeFile("1,2,3\n");
	EXPECT_FALSE(read());
}

// A blank line with a Windows line ending is read as a row of zeros.
TEST_F(DtrgMixerCsv, DISABLED_BlankCrlfLineIsSkipped)
{
	writeFile("\r\n" + rows(1, "\r\n"));
	ASSERT_TRUE(read());
	expectRows(1);
}

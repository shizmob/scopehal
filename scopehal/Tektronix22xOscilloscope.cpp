/***********************************************************************************************************************
*                                                                                                                      *
* libscopehal                                                                                                          *
*                                                                                                                      *
* Copyright (c) 2026 Shiz <hi@shiz.me>                                                                                 *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

#include <cstring>
#include <ios>
#include <iomanip>

#include "scopehal.h"
#include "Tektronix22xOscilloscope.h"
#include "EdgeTrigger.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constants and types

static constexpr size_t TX_TIMEOUT = 50 * 1000;
static constexpr size_t RX_TIMEOUT = 1000 * 1000;
static constexpr size_t RX_CHECK_TIMEOUT = 100 * 1000;
static constexpr size_t RX_ACQ_TIMEOUT = 5000 * 1000;
static constexpr size_t NUM_CHANNELS = 2;
// 222: Tektronix document 070-7100-00, page 8-1
// 224: Tektronix document 070-8476-00, page D-3
static constexpr float NUM_VERTICAL_DIVS = 10.24;
// 222: Tektronix document 070-7100-00, page 8-7
// 224: Tektronix document 070-8476-00, page D-14
static constexpr float NUM_HORIZONTAL_DIVS = 10.24;

enum Chan : size_t
{
	CH_1 = 0,
	CH_2 = 1,
	CH_EXT = 2,
};

enum FPCategoryOffset
{
	FP_CH1 = 0,
	FP_CH2 = 8,
	FP_HOR = 16,
	FP_TRIG = 24,
	FP_MISC = 32,
};

enum FPCHFieldOffset
{
	FP_CH_SCALE = 0,
	FP_CH_COUPLING = 4,
	FP_CH_CAL = 6,
	FP_CH_INVERT = 7,
};

static constexpr uint32_t FP_CH_SCALES[] =
{
	5, 10, 20,
	50, 100, 200,
	500, 1000, 2000,
	5000, 10000, 20000,
	50000, 100000, 200000,
	500000,
};

enum FPCoupling : uint64_t
{
	FP_CH_COUPLING_DC = 0b00ull,
	FP_CH_COUPLING_AC = 0b01ull,
	FP_CH_COUPLING_GND = 0b10ull,
	FP_CH_COUPLING_OFF = 0b11ull,
};

enum FPHorFieldOffset
{
	FP_HOR_SCALE = 0,
	FP_HOR_X10 = 5,
	FP_HOR_XY = 6,
	FP_HOR_RO = 7,
};

static constexpr uint64_t FP_HOR_SCALES[] =
{
	50ull, 100ull, 200ull,
	500ull, 1000ull, 2000ull,
	5000ull, 10000ull, 20000ull,
	50000ull, 100000ull, 200000ull,
	500000ull, 1000000ull, 2000000ull,
	5000000ull, 10000000ull, 20000000ull,
	50000000ull, 100000000ull, 200000000ull,
	500000000ull, 1000000000ull, 2000000000ull,
	5000000000ull, 10000000000ull, 20000000000ull,
};

enum FPTrigFieldOffset
{
	FP_TRIG_MODE = 0,
	FP_TRIG_SRC = 3,
	FP_TRIG_SLOPE = 5,
	FP_TRIG_POS = 6,
};

enum FPTrigMode : uint64_t
{
	FP_TRIG_MODE_NORM = 0ull,
	FP_TRIG_MODE_AUTO_LVL = 1ull,
	FP_TRIG_MODE_AUTO_BL = 2ull,
	FP_TRIG_MODE_SSEQ = 3ull,
};

enum FPTrigSource : uint64_t
{
	FP_TRIG_SRC_VERT = 0ull,
	FP_TRIG_SRC_CH1 = 1ull,
	FP_TRIG_SRC_CH2 = 2ull,
	FP_TRIG_SRC_EXT = 3ull,
};

enum FPTrigSlope : uint64_t
{
	FP_TRIG_SLOPE_FALLING = 0ull,
	FP_TRIG_SLOPE_RISING = 1ull,
};

enum FPTrigPos : uint64_t
{
	FP_TRIG_POS_POST = 0ull,
	FP_TRIG_POS_MID = 1ull,
	FP_TRIG_POS_PRE = 2ull,
};

enum FPMiscFieldOffset
{
	FP_MISC_AUTO_TRIG = 0,
	FP_MISC_STORE = 1,
	FP_MISC_MODE = 2,
	FP_MISC_STORE_VALID = 4,
	FP_MISC_WFM_RCL = 5,
	FP_MISC_CHAN_SEL = 6,
	FP_MISC_TIMEOUT = 7,
};

enum FPAcqMode : uint64_t
{
	FP_ACQ_MODE_NORM = 0ull,
	FP_ACQ_MODE_ENV = 1ull,
	FP_ACQ_MODE_AVG = 2ull,
	FP_ACQ_MODE_CENV = 3ull,
};

enum FPAcqChannel : uint64_t
{
	FP_ACQ_CH2 = 0ull,
	FP_ACQ_CH1 = 1ull,
};

// 222: Tektronix document 070-7533-00, page 6, table 1
// 224: Tektronix document 070-8476-00, page B-9, table B-2
constexpr static size_t DAC_HOR_POS_SCALE_DIV = 5;
constexpr static size_t DAC_CH_TRIG_SCALE_DIV = 30;
constexpr static float  DAC_EXT_TRIG_SCALE_V = 2.33;
constexpr static float  DAC_CH_GAIN_SCALE = 2.5;
constexpr static size_t DAC_CH_POS_SCALE_DIV = 12;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Utilities

optional<string> RemovePrefix(const std::string &s, const char *prefix)
{
	size_t len = strlen(prefix);
	if (strncmp(s.data(), prefix, len) == 0)
		return s.substr(len);
	return {};
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

Tektronix22xOscilloscope::Tektronix22xOscilloscope(SCPITransport* transport)
	: SCPIDevice(transport, /* identify */ false)
	, SCPIInstrument(transport)
{
	m_transport->SetTimeouts(TX_TIMEOUT, RX_TIMEOUT);

	// Wait for oscilloscope to be ready
	auto reply = SendCommandWithResponse("STA?");
	if (!reply)
		LogWarning("Failed to wait for oscilloscope to be ready\n");
	else if (reply.value() != "READY")
		LogWarning("Unrecognized oscilloscope status: %s\n", reply.value().c_str());

	// Obtain oscilloscope identity (IDPing() updates the internal parameters)
	(void)IDPing();

	// Add the base channels
	auto ch1 = new OscilloscopeChannel(
		this, "CH1", "#4040ff", Unit(Unit::UNIT_FS), Unit(Unit::UNIT_VOLTS), Stream::STREAM_TYPE_ANALOG, CH_1);
	m_channels.push_back(ch1);
	ch1->SetDefaultDisplayName();

	auto ch2 = new OscilloscopeChannel(
		this, "CH2", "#ff4040", Unit(Unit::UNIT_FS), Unit(Unit::UNIT_VOLTS), Stream::STREAM_TYPE_ANALOG, CH_2);
	m_channels.push_back(ch2);
	ch2->SetDefaultDisplayName();

	// Add the external trigger input
	m_extTrigChannel = new OscilloscopeChannel(
		this, "EXT", "", Unit(Unit::UNIT_FS), Unit(Unit::UNIT_VOLTS), Stream::STREAM_TYPE_TRIGGER, CH_EXT);
	m_channels.push_back(m_extTrigChannel);
	m_extTrigChannel->SetDefaultDisplayName();

	// Create Vulkan objects for the waveform conversion
	InitVulkanQueue("Tektronix22xOscilloscope");
}

Tektronix22xOscilloscope::~Tektronix22xOscilloscope()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Driver interface functions

string Tektronix22xOscilloscope::GetDriverNameInternal()
{
	return "tektronix.22x";
}

vector<SCPIInstrumentModel> Tektronix22xOscilloscope::GetDriverSupportedModels()
{
	return
	{
		{"Tektronix 222/224",
		{
			{ SCPITransportType::TRANSPORT_UART, "/dev/tty<x>" },
		}},
	};
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Command functions

optional<Tektronix22xOscilloscope::Error> Tektronix22xOscilloscope::SendCommand(const string &cmd, const string &arg)
{
	LogTrace(">> %s(%s)\n", cmd.c_str(), arg.c_str());
	lock_guard<recursive_mutex> lock(m_transport->GetMutex());
	string s;

	if (!arg.empty())
	{
		s = cmd + " ";
		m_transport->SendRawData(s.size(), reinterpret_cast<uint8_t *>(s.data()));

		// Check if the command is bad, if so the instrument will tell us early and abort processing
		m_transport->SetTimeouts(TX_TIMEOUT, RX_CHECK_TIMEOUT);
		s.clear();
		s.reserve(1);
		size_t nr = m_transport->ReadRawData(1, reinterpret_cast<uint8_t *>(s.data()));
		m_transport->SetTimeouts(TX_TIMEOUT, RX_TIMEOUT);

		// Error indication, parse it
		if (nr > 0)
		{
			s += m_transport->ReadReply(false);
			auto maybeStatus = RemovePrefix(s, "STA ");
			if (maybeStatus)
			{
				Error error{static_cast<uint16_t>(stoi(maybeStatus.value(), nullptr, 16))};
				LogError("Command %s failed: 0x%04X\n", cmd.c_str(), error);
				return error;
			}
			else
			{
				LogError("Command %s failed: %s\n", cmd.c_str(), s.c_str());
				return Error{0};
			}
		}

		// All good, send the rest
		s = arg + "\r";
		m_transport->SendRawData(s.size(), reinterpret_cast<uint8_t *>(s.data()));
	}
	else
	{
		s = cmd + "\r";
		m_transport->SendRawData(s.size(), reinterpret_cast<uint8_t *>(s.data()));
	}

	return {};
}

optional<string> Tektronix22xOscilloscope::SendCommandWithResponse(const string &cmd, const string& arg)
{
	lock_guard<recursive_mutex> lock(m_transport->GetMutex());

	auto maybeError = SendCommand(cmd, arg);
	if (maybeError)
	{
		LogError("Command %s failed: 0x%04X\n", cmd.c_str(), maybeError.value());
		return {};
	}

	string reply = m_transport->ReadReply(false);
	LogTrace("<< %s\n", reply.c_str());

	// Remove trailers
	while (!reply.empty())
	{
		char ch = reply[reply.size() - 1];
		if (ch != '\r' && ch != '\n' && ch != ';')
		{
			break;
		}
		reply.pop_back();
	}
	// Check for error status
	auto maybeStatus = RemovePrefix(reply, "STA ");
	if (maybeStatus)
	{
		int error = stoi(maybeStatus.value(), nullptr, 16);
		LogError("Command %s failed: 0x%04X\n", cmd.c_str(), error);
		return {};
	}

	// Strip command from reply
	auto cmdPrefix = cmd;
	if (!cmdPrefix.empty() && cmdPrefix[cmdPrefix.size() - 1] == '?')
		cmdPrefix.pop_back();
	cmdPrefix.push_back(' ');
	auto maybeData = RemovePrefix(reply, cmdPrefix.c_str());
	// Sometimes, a reply is just e.g. "READY", so return that otherwise
	return maybeData ? *maybeData : reply;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Button functions

static constexpr const char *BUTTON_NAMES[] =
{
	nullptr,
	/* BUT_CLEAR */ "1",
	/* BUT_MENU0 */ "2",
	/* BUT_MENU1 */ "3",
	/* BUT_MENU2 */ "4",
	/* BUT_MENU3 */ "5",
	/* BUT_OFF   */ "6",
	nullptr,
	nullptr,
	/* BUT_TRIG_SRC   */ "9",
	/* BUT_TRIG_MODE  */ "A",
	/* BUT_TRIG_SLOPE */ "B",
	/* BUT_CH2        */ "C",
	/* BUT_CH1        */ "D",
	/* BUT_AUTO_SETUP */ "E",
	nullptr,
	nullptr,
	/* BUT_SETUP    */ "11",
	/* BUT_TRIG_POS */ "12",
	/* BUT_AUX_FUNC */ "13",
	/* BUT_DISPL    */ "14",
	nullptr,
	nullptr,
	nullptr,
	nullptr,
	/* BUT_WAVE_SAVE  */ "19",
	/* BUT_WAVE_RCL   */ "1A",
	/* BUT_STORE_MODE */ "1B",
	/* BUT_ACQ_MODE   */ "1C",
	nullptr,
	nullptr,
	nullptr,
	/* BUT_X10_MAG  */ "20",
	/* BUT_VAR      */ "21",
	/* BUT_AUTO_LVL */ "22",
};

void Tektronix22xOscilloscope::PushButton(Button i)
{
	SendCommand("BUT", BUTTON_NAMES[i]);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DAC functions

static constexpr const char *DAC_NAMES[] =
{
	/* DAC_HOR_POS        */ "00",
	/* DAC_CH1_TRIG_LEVEL */ "01",
	/* DAC_CH2_TRIG_LEVEL */ "02",
	/* DAC_EXT_TRIG_LEVEL */ "03",
	/* DAC_CH2_GAIN       */ "04",
	/* DAC_CH1_GAIN       */ "05",
	/* DAC_CH2_POS        */ "06",
	/* DAC_CH1_POS        */ "07",
};

uint16_t Tektronix22xOscilloscope::QueryDAC(DAC i)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if (m_dacValues.find(i) != m_dacValues.end())
			return m_dacValues[i];
	}

	string dacName = DAC_NAMES[i];
	auto reply = SendCommandWithResponse("DAC?", dacName);
	if (!reply)
	{
		LogError("Querying DAC %s failed\n", dacName.c_str());
		return 0;
	}
	string rvalue = reply.value();
	if (rvalue.find(dacName + ":") != 0)
	{
		LogError("Querying FP %s returned weird value: \"%s\"\n", dacName.c_str(), rvalue.c_str());
		return 0;
	}

	string state = rvalue.substr(dacName.size() + 1);
	uint16_t st = stoi(state, nullptr, 16);

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_dacValues[i] = st;
	return st;
}

void Tektronix22xOscilloscope::SetDAC(DAC i, uint16_t val)
{
	string dacName = DAC_NAMES[i];
	stringstream arg;
	arg << dacName << ":" << hex << uppercase << setw(4) << setfill('0') << val;
	auto reply = SendCommandWithResponse("DAC", arg.str());
	if (!reply)
	{
		LogFatal("Setting DAC %s failed\n", dacName.c_str());
	}
	string rvalue = reply.value();
	if (rvalue != "READY")
	{
		LogFatal("Setting DAC %s returned weird value: \"%s\"\n", dacName.c_str(), rvalue.c_str());
		return;
	}

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_dacValues[i] = val;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FP functions

static constexpr const char *FP_NAMES[] =
{
	/* FP_ACQ  */ "ACQ",
	/* FP_REF1 */ "REF1",
	/* FP_REF2 */ "REF2",
	/* FP_REF3 */ "REF3",
	/* FP_REF4 */ "REF4",
	/* FP_STR1 */ "STR1",
	/* FP_STR2 */ "STR2",
	/* FP_STR3 */ "STR3",
	/* FP_STR4 */ "STR4",
};

uint64_t Tektronix22xOscilloscope::QueryFP(FP i)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if (m_fpValues.find(i) != m_fpValues.end())
			return m_fpValues[i];
	}

	string fpName = FP_NAMES[i];
	auto reply = SendCommandWithResponse("FP?", fpName);
	if (!reply)
	{
		LogFatal("Querying FP %s failed\n", fpName.c_str());
	}
	string rvalue = reply.value();
	if (rvalue.find(fpName + ":") != 0)
	{
		LogFatal("Querying FP %s returned weird value: \"%s\"\n", fpName.c_str(), rvalue.c_str());
	}

	size_t stateOffset = fpName.size() + 1;
	uint64_t val = 0;
	for (size_t j = 0; j < 5; j++)
	{
		uint64_t b = stoi(rvalue.substr(stateOffset + j * 2, 2), nullptr, 16);
		val |= b << (j * 8);
	}

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_fpValues[i] = val;
	return val;
}

void Tektronix22xOscilloscope::SetFP(FP i, uint64_t val)
{
	string fpName = FP_NAMES[i];
	stringstream arg;
	arg << fpName << ":";
	for (size_t j = 0; j < 5; j++)
	{
		arg << hex << uppercase << setw(2) << setfill('0') << (val & 0xff);
		val >>= 8;
	}
	auto reply = SendCommandWithResponse("FP", arg.str());
	if (!reply)
	{
		LogFatal("Setting FP %s failed\n", fpName.c_str());
	}
	string rvalue = reply.value();
	if (rvalue != "READY")
	{
		LogFatal("Setting FP %s returned weird value: \"%s\"\n", fpName.c_str(), rvalue.c_str());
	}

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_fpValues[i] = val;
}

void Tektronix22xOscilloscope::AdjustFP(FP i, uint8_t cat, uint8_t val, uint8_t mask)
{
	uint64_t fpVal = QueryFP(i);
	uint64_t newVal = (uint64_t)val << cat;
	uint64_t newMask = (uint64_t)mask << cat;
	if ((fpVal & newMask) != newVal)
	{
		fpVal &= ~newMask;
		fpVal |= (newVal & newMask);
		SetFP(i, fpVal);
	}
}

uint8_t Tektronix22xOscilloscope::QueryFPChannel(size_t i)
{
	size_t field;
	switch(i)
	{
	case CH_1:
		field = FP_CH1;
		break;
	case CH_2:
		field = FP_CH2;
		break;
	default:
		LogError("Can not query FP configuration for invalid channel %zu\n", i);
		return 0;
	}

	uint64_t fp = QueryFP(FP_ACQ);
	return (uint8_t)((fp >> field) & 0xff);
}

void Tektronix22xOscilloscope::AdjustFPChannel(size_t i, uint8_t value, uint8_t mask)
{
	uint8_t field;
	switch(i)
	{
	case CH_1:
		field = FP_CH1;
		break;
	case CH_2:
		field = FP_CH2;
		break;
	default:
		LogError("Can not adjust FP configuration for invalid channel %zu\n", i);
		return;
	}

	AdjustFP(FP_ACQ, field, value, mask);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Device interface functions

std::string Tektronix22xOscilloscope::IDPing()
{
	// Obtain oscilloscope identity
	auto maybeReply = SendCommandWithResponse("ID?");
	if (!maybeReply)
	{
		LogWarning("Failed to obtain oscilloscope identity\n");
		return "";
	}
	auto maybeId = RemovePrefix(maybeReply.value(), "TEK-");
	if (!maybeId)
	{
		LogWarning("Unrecognized oscilloscope identity: %s\n", maybeReply.value().c_str());
		return "";
	}

	auto id = maybeId.value();
	m_vendor = "Tektronix";

	auto pos = id.find(' ');
	if (pos == string::npos)

		pos = id.size();

	m_model = id.substr(0, pos);
	if (m_model == "222")
		m_modelId = MODEL_222;
	else if (m_model == "222A")
		m_modelId = MODEL_222A;
	else if (m_model == "222PS")
		m_modelId = MODEL_222PS;
	else if (m_model == "224")
		m_modelId = MODEL_224;
	else
	{
		LogWarning("Unrecognized oscilloscope model: %s\n", m_model.c_str());
		m_modelId = MODEL_UNKNOWN;
	}

	while (++pos < id.size())
	{
		auto nextPos = id.find(' ', pos);
		if (nextPos == string::npos)
			nextPos = id.size();

		auto chunk = id.substr(pos, nextPos - pos);
		auto ver = RemovePrefix(chunk, "VER:");
		auto sn = RemovePrefix(chunk, "SN:");
		if (ver)
			m_fwVersion = ver.value();
		else if (sn)
			m_serial = sn.value();
		else
			LogWarning("Unrecognized oscilloscope identity field: %s\n", chunk.c_str());
		pos = nextPos;
	}

	return id;
}

unsigned int Tektronix22xOscilloscope::GetInstrumentTypes() const
{
	return Instrument::INST_OSCILLOSCOPE;
}

uint32_t Tektronix22xOscilloscope::GetInstrumentTypesForChannel(size_t /*i*/) const
{
	return Instrument::INST_OSCILLOSCOPE;
}

void Tektronix22xOscilloscope::FlushConfigCache()
{
	lock_guard<recursive_mutex> lock(m_cacheMutex);

	m_fpValues.clear();
	m_dacValues.clear();
	m_channelCouplings.clear();

	delete m_trigger;
	m_trigger = NULL;
}

bool Tektronix22xOscilloscope::IsChannelEnabled(size_t i)
{
	switch(i)
	{
	case CH_1:
	case CH_2:
	{
		uint8_t ch_fp = QueryFPChannel(i);
		return ((ch_fp >> FP_CH_COUPLING) & 0b11) != FP_CH_COUPLING_OFF;
	}
	case CH_EXT:
		return false;
	default:
		LogError("Can not query status for invalid channel %zu\n", i);
		return false;
	}
}

void Tektronix22xOscilloscope::EnableChannel(size_t i)
{
	switch(i)
	{
	case CH_1:
	case CH_2:
	{
		OscilloscopeChannel::CouplingType coupling = GetChannelCoupling(i);

		uint8_t ch_fp;
		switch (coupling)
		{
		case OscilloscopeChannel::COUPLE_DC_1M:
			ch_fp = FP_CH_COUPLING_DC;
			break;
		case OscilloscopeChannel::COUPLE_AC_1M:
			ch_fp = FP_CH_COUPLING_AC;
			break;
		case OscilloscopeChannel::COUPLE_GND:
			ch_fp = FP_CH_COUPLING_GND;
			break;
		default:
			LogError("Invalid coupling for channel %zu\n", i);
			return;
		}
		AdjustFPChannel(i, ch_fp << FP_CH_COUPLING, 0b11 << FP_CH_COUPLING);
		break;
	}
	case CH_EXT:
		break;
	default:
		LogError("Can not enable invalid channel %zu\n", i);
		return;
	}
}

void Tektronix22xOscilloscope::DisableChannel(size_t i)
{
	switch(i)
	{
	case CH_1:
	case CH_2:
		AdjustFPChannel(i, FP_CH_COUPLING_OFF << FP_CH_COUPLING, 0b11 << FP_CH_COUPLING);
		break;
	case CH_EXT:
		LogError("Can not disable external trigger channel\n");
		return;
	default:
		LogError("Can not disable invalid channel %zu\n", i);
		return;
	}
}

OscilloscopeChannel* Tektronix22xOscilloscope::GetExternalTrigger()
{
	return m_extTrigChannel;
}

vector<OscilloscopeChannel::CouplingType> Tektronix22xOscilloscope::GetAvailableCouplings(size_t i)
{
	vector<OscilloscopeChannel::CouplingType> ret;
	switch(i)
	{
	case CH_1:
	case CH_2:
		ret.push_back(OscilloscopeChannel::COUPLE_DC_1M);
		ret.push_back(OscilloscopeChannel::COUPLE_AC_1M);
		ret.push_back(OscilloscopeChannel::COUPLE_GND);
		break;
	}
	return ret;
}

OscilloscopeChannel::CouplingType Tektronix22xOscilloscope::GetChannelCoupling(size_t i)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if (m_channelCouplings.find(i) != m_channelCouplings.end())
			return m_channelCouplings[i];
	}

	OscilloscopeChannel::CouplingType coupling;
	switch(i)
	{
	case CH_1:
	case CH_2:
	{
		auto ch_fp = QueryFPChannel(i);
		switch ((ch_fp >> FP_CH_COUPLING) & 0b11)
		{
		case FP_CH_COUPLING_DC:
			coupling = OscilloscopeChannel::COUPLE_DC_1M;
			break;
		case FP_CH_COUPLING_AC:
			coupling = OscilloscopeChannel::COUPLE_AC_1M;
			break;
		default:
			coupling = OscilloscopeChannel::COUPLE_GND;
			break;
		}
		break;
	}
	case CH_EXT:
		coupling = OscilloscopeChannel::COUPLE_SYNTHETIC;
		break;
	default:
		LogError("Can not get coupling of invalid channel %zu\n", i);
		return OscilloscopeChannel::COUPLE_SYNTHETIC;
	}

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelCouplings[i] = coupling;
	return coupling;
}

void Tektronix22xOscilloscope::SetChannelCoupling(size_t i, OscilloscopeChannel::CouplingType type)
{
	m_channelCouplings[i] = type;

	if (IsChannelEnabled(i))
		// Coupling and on/off status are linked
		EnableChannel(i);
}

double Tektronix22xOscilloscope::GetChannelAttenuation(size_t /*i*/)
{
	switch (m_modelId)
	{
	case MODEL_UNKNOWN:
	case MODEL_222:
		// Tektronix document 070-7100-00, page 6-3:
		// "The probe attenuation factor is 3 and is automatically reflected in the VOLTS/DIV readouts."
		return 3;
	case MODEL_222A:
	case MODEL_222PS:
	case MODEL_224:
		// 222PS: Tektronix document 070-8097-02, page 3-43:
		// 224:   Tektronix document 070-8476-00, page 3-43:
		// "Actual probe attenuation factors are 3X for the P400 probe and 30X for the P8S0 probe."
		// Sadly, there seems to be no way to read out the current probe setting...
		return 3;
	}
}

void Tektronix22xOscilloscope::SetChannelAttenuation(size_t i, double atten)
{
	// Not configurable
	if (atten != GetChannelAttenuation(i))
	{
		LogError("Invalid attenuation for channel %zu: %lf\n", i, atten);
	}
}

vector<unsigned int> Tektronix22xOscilloscope::GetChannelBandwidthLimiters(size_t /*i*/)
{
	return {0};
}

unsigned int Tektronix22xOscilloscope::GetChannelBandwidthLimit(size_t /*i*/)
{
	return 0;
}

void Tektronix22xOscilloscope::SetChannelBandwidthLimit(size_t i, unsigned int limit_mhz)
{
	// Not configurable
	if (limit_mhz != GetChannelBandwidthLimit(i))
	{
		LogError("Invalid bandwidth limit for channel %zu: %u\n", i, limit_mhz);
	}
}

float Tektronix22xOscilloscope::GetChannelVoltageRange(size_t i, size_t /*stream*/)
{
	switch(i)
	{
	case CH_1:
	case CH_2:
	{
		auto chFp = QueryFPChannel(i);
		auto chScaleIndex = (chFp >> FP_CH_SCALE) & 0b1111;
		auto divScale = FP_CH_SCALES[chScaleIndex] / 1000.f;
		return divScale * NUM_VERTICAL_DIVS;
		break;
	}
	case CH_EXT:
		return 0.f;
	default:
		LogError("Can not get voltage range for invalid channel %zu\n", i);
		return 0.f;
	}

}

void Tektronix22xOscilloscope::SetChannelVoltageRange(size_t i, size_t /*stream*/, float range)
{
	switch(i)
	{
	case CH_1:
	case CH_2:
		for (size_t j = 0; j < sizeof(FP_CH_SCALES) / sizeof(FP_CH_SCALES[0]); j++)
		{
			if (range == (FP_CH_SCALES[j] / 1000.f) * NUM_VERTICAL_DIVS)
			{
				AdjustFPChannel(i, j << FP_CH_SCALE, 0b1111 << FP_CH_SCALE);
				return;
			}
		}
		LogError("Invalid voltage range for channel %zu: %f\n", i, range);
		break;
	case CH_EXT:
		LogError("Can not set voltage range for external trigger channel\n");
		break;
	default:
		LogError("Can not set voltage range for invalid channel %zu\n", i);
		break;
	}
}

bool Tektronix22xOscilloscope::CanAverage(size_t i)
{
	switch (i)
	{
	case CH_1:
	case CH_2:
		return true;
	default:
		return false;
	}
}

size_t Tektronix22xOscilloscope::GetNumAverages(size_t i)
{
	switch (i)
	{
	case CH_1:
	case CH_2:
	{
		uint8_t misc = (QueryFP(FP_ACQ) >> FP_MISC) & 0xff;
		switch ((misc >> FP_MISC_MODE) & 0b11)
		{
		case FP_ACQ_MODE_NORM:
		case FP_ACQ_MODE_ENV:
		case FP_ACQ_MODE_CENV:
			return 1;
		case FP_ACQ_MODE_AVG:
			return 4;
		}
	}
	case CH_EXT:
		LogError("Can not get average count for external trigger channel\n");
		return  0;;
	default:
		LogError("Can not get average count for invalid channel %zu\n", i);
		return 0;
	}
}

void Tektronix22xOscilloscope::SetNumAverages(size_t i, size_t navg)
{
	switch (i)
	{
	case CH_1:
	case CH_2:
	{
		uint8_t mode;
		switch (navg)
		{
		case 1:
			mode = FP_ACQ_MODE_NORM;
			break;
		case 4:
			mode = FP_ACQ_MODE_AVG;
			break;
		default:
			LogError("Invalid average count for channel %zu: %zu\n", i, navg);
			return;
		}
		AdjustFP(FP_ACQ, FP_MISC, (mode << FP_MISC_MODE), (0b11 << FP_MISC_MODE));
		break;
	}
	case CH_EXT:
		LogError("Can not set average count for external trigger channel\n");
		return;
	default:
		LogError("Can not set average count for invalid channel %zu\n", i);
		return;
	}
}

float Tektronix22xOscilloscope::GetChannelOffset(size_t i, size_t stream)
{
	uint16_t pos;
	switch(i)
	{
	case CH_1:
		pos = QueryDAC(DAC_CH1_POS);
		break;
	case CH_2:
		pos = QueryDAC(DAC_CH2_POS);
		break;
	case CH_EXT:
		LogError("Can not get offset for external trigger channel\n");
		return 0.f;
	default:
		LogError("Can not get offset for invalid channel %zu\n", i);
		return 0.f;
	}

	float mag = ((int32_t)pos - 4095) / 4095.f;
	return mag * GetChannelVoltageRange(i, stream) / NUM_VERTICAL_DIVS * (DAC_CH_POS_SCALE_DIV * 2);
}

void Tektronix22xOscilloscope::SetChannelOffset(size_t i, size_t stream, float offset)
{
	enum DAC dac;
	switch(i)
	{
	case CH_1:
		dac = DAC_CH1_POS;
		break;
	case CH_2:
		dac = DAC_CH2_POS;
		break;
	case CH_EXT:
		LogError("Can not set offset for external trigger channel\n");
		return;
	default:
		LogError("Can not set offset for invalid channel %zu\n", i);
		return;
	}

	float mag = offset / (GetChannelVoltageRange(i, stream) / NUM_VERTICAL_DIVS * (DAC_CH_POS_SCALE_DIV * 2));
	uint16_t pos_val = round(mag * 4095.f) + 4095;
	SetDAC(dac, pos_val);
}

bool Tektronix22xOscilloscope::CanInvert(size_t i)
{
	switch (i)
	{
	case CH_1:
	case CH_2:
		return true;
	default:
		return false;
	}
}

void Tektronix22xOscilloscope::Invert(size_t i, bool invert)
{
	switch (i)
	{
	case CH_1:
	case CH_2:
	{
		uint8_t mode = (invert ? 1 : 0) << FP_CH_INVERT;
		AdjustFPChannel(i, mode, 1 << FP_CH_INVERT);
	}
	case CH_EXT:
		LogError("Can not set inversion for external trigger channel\n");
		return;
	default:
		LogError("Can not set inversion for invalid channel %zu\n", i);
		return;
	}
}

bool Tektronix22xOscilloscope::IsInverted(size_t i)
{
	switch (i)
	{
	case CH_1:
	case CH_2:
		return ((QueryFPChannel(i) >> FP_CH_INVERT) & 1) != 0;
	case CH_EXT:
		return false;
	default:
		LogError("Can not get inversion for invalid channel %zu\n", i);
		return false;
	}
}

Oscilloscope::TriggerMode Tektronix22xOscilloscope::PollTrigger()
{
	auto reply = SendCommandWithResponse("TRG?");
	if (!reply)
	{
		LogError("Querying trigger failed\n");
		return TRIGGER_MODE_STOP;
	}

	string rvalue = reply.value();
	if (rvalue == "YES")
	{
		// We potentially get triggered while the trace is still running,
		// without an interface to wait for it to finish.
		// If we request data before the acquisition is done, we get zeroes.
		// Instead, we note when the acquisition started and wait for the duration
		// of the remaining trace length, and *then* indicate we're done.
		double remaining;
		if (!m_acqDoneTime)
		{
			uint64_t traceLength = static_cast<uint64_t>(QueryHorizontalScale() * FS_PER_NANOSECOND * NUM_HORIZONTAL_DIVS);
			int64_t offset = GetTriggerOffset();
			remaining = 5 * static_cast<double>(traceLength - offset) / FS_PER_SECOND;
			m_acqDoneTime = GetTime() + remaining;
		}
		else
		{
			remaining = *m_acqDoneTime - GetTime();
		}
		// If the remaining time is really low, proceed anyway as the delay to download the data
		// will make up for it.
		return (remaining < 100e-6) ? TRIGGER_MODE_TRIGGERED : TRIGGER_MODE_RUN;
	}
	else if (rvalue == "NO")
	{
		return TRIGGER_MODE_RUN;
	}
	else
	{
		LogError("Querying trigger returned unknown value: %s\n", rvalue.c_str());
		return TRIGGER_MODE_STOP;
	}
}

uint64_t Tektronix22xOscilloscope::QueryHorizontalScale()
{
	uint8_t hor = (QueryFP(FP_ACQ) >> FP_HOR) & 0xff;
	uint8_t scaleIndex = (hor >> FP_HOR_SCALE) & 0b11111;
	return FP_HOR_SCALES[scaleIndex];
}

void Tektronix22xOscilloscope::SetHorizontalScale(uint64_t scale)
{
	for (size_t i = 0; i < sizeof(FP_HOR_SCALES) / sizeof(FP_HOR_SCALES[0]); i++)
	{
		// Set to first scale equivalent or larger than requested.
		if (scale >= FP_HOR_SCALES[i])
		{
			AdjustFP(FP_ACQ, FP_HOR, i << FP_HOR_SCALE, 0b11111 << FP_HOR_SCALE);
			break;
		}
	}
}

bool Tektronix22xOscilloscope::AcquireData()
{
	// Notify about download operation start
	ChannelsDownloadStarted();

	// Scopes do not have a capture time so we fake it
	double now = GetTime();

	SequenceSet seq;
	m_transport->SetTimeouts(TX_TIMEOUT, RX_ACQ_TIMEOUT);
	for (size_t i = 0; i < NUM_CHANNELS; i++)
	{
		if (!IsChannelEnabled(i))
			continue;

		// Download data
		auto channame = GetChannel(i)->GetHwname();
		ChannelsDownloadStatusUpdate(i, InstrumentChannel::DownloadState::DOWNLOAD_IN_PROGRESS, 0.0);
		auto reply = SendCommandWithResponse("CURV?", channame);
		if (!reply)
		{
			LogError("Acquiring channel %s data failed\n", channame.c_str());
			continue;
		}
		ChannelsDownloadStatusUpdate(i, InstrumentChannel::DownloadState::DOWNLOAD_FINISHED, 1.0);
		string rvalue = reply.value();
		if (rvalue.find(channame + ":") != 0)
		{
			LogError("Acquiring channel %s data returned weird value: %s\n", channame.c_str(), rvalue.c_str());
			continue;
		}
		string data = rvalue.substr(channame.size() + 1);

		// Gather basic parameters
		uint16_t xcount = stoi(data.substr(12, 4), nullptr, 16);
		auto yincrement = GetChannelVoltageRange(i, 0) / 256.0f;
		auto yorigin = GetChannelOffset(i, 0);
		auto xincrement = round(QueryHorizontalScale() * FS_PER_NANOSECOND * (NUM_HORIZONTAL_DIVS / xcount));
		if (data.size() < 16 + xcount * 2 + 2)
		{
			LogError("Truncated trace for channel %s, ignoring", channame.c_str());
			continue;
		}

		// Store data
		auto cap = AllocateAnalogWaveform(m_nickname + "." + channame);
		cap->Resize(xcount);
		cap->m_timescale = xincrement;
		cap->m_triggerPhase = 0;
		cap->m_startTimestamp = floor(now);
		cap->m_startFemtoseconds = (now - floor(now)) * FS_PER_SECOND;

		cap->PrepareForCpuAccess();
		for (size_t j = 0; j < xcount; j ++)
		{
			int16_t sample = stoi(data.substr(16 + j * 2, 2), nullptr, 16);
			cap->m_samples[j] = (float)(sample - 128) * yincrement - yorigin;
		}
		cap->MarkSamplesModifiedFromCpu();

		seq[GetOscilloscopeChannel(i)] = cap;
	}

	// Put data into pending wavefrms
	{
		lock_guard<mutex> lock(m_pendingWaveformsMutex);
		m_pendingWaveforms.push_back(seq);
	}

	// Tell the download monitor that waveform download has finished
	ChannelsDownloadFinished();

	// Re-arm trigger
	if (m_rearmTrigger)
		ArmTrigger();

	return true;
}

void Tektronix22xOscilloscope::ArmTrigger()
{
	// "INIT" button rearms trigger
	PushButton(BUT_MENU3);
	m_triggerArmed = true;
	m_acqDoneTime.reset();
}

void Tektronix22xOscilloscope::Start()
{
	StartSingleTrigger();
	m_rearmTrigger = true;
}

void Tektronix22xOscilloscope::StartSingleTrigger()
{
	// Set single trigger
	AdjustFP(FP_ACQ, FP_TRIG, FP_TRIG_MODE_SSEQ << FP_TRIG_MODE, 0b111 << FP_TRIG_MODE);
	ArmTrigger();
	m_rearmTrigger = false;
}

void Tektronix22xOscilloscope::Stop()
{
	// TODO: no stop mode?
	m_rearmTrigger = false;
	m_triggerArmed = false;
	m_acqDoneTime.reset();
}

void Tektronix22xOscilloscope::ForceTrigger()
{
	LogError("Tektronix22xOscilloscope::ForceTrigger not implemented for this model\n");
}

bool Tektronix22xOscilloscope::IsTriggerArmed()
{
	return m_triggerArmed;
}

void Tektronix22xOscilloscope::PushTrigger()
{
	auto et = dynamic_cast<EdgeTrigger*>(m_trigger);
	if (!et)
	{
		LogError("Unsupported trigger type (not an edge)\n");
		return;
	}

	uint8_t fpVal = 0;
	uint8_t fpMask = (0b11 << FP_TRIG_SRC) | (1 << FP_TRIG_SLOPE);

	// Source
	InstrumentChannel *src = et->GetInput(0).m_channel;
	double levelScale;
	DAC levelDac;
	switch (src->GetIndex())
	{
	case CH_1:
		fpVal |= (FP_TRIG_SRC_CH1 << FP_TRIG_SRC);
		levelDac = DAC_CH1_TRIG_LEVEL;
		levelScale = GetChannelVoltageRange(CH_1, 0) / NUM_VERTICAL_DIVS * DAC_CH_TRIG_SCALE_DIV;
		break;
	case CH_2:
		fpVal |= (FP_TRIG_SRC_CH2 << FP_TRIG_SRC);
		levelDac = DAC_CH2_TRIG_LEVEL;
		levelScale = GetChannelVoltageRange(CH_2, 0) / NUM_VERTICAL_DIVS * DAC_CH_TRIG_SCALE_DIV;
		break;
	case CH_EXT:
		fpVal |= (FP_TRIG_SRC_EXT << FP_TRIG_SRC);
		levelDac = DAC_EXT_TRIG_LEVEL;
		levelScale = DAC_EXT_TRIG_SCALE_V;
		break;
	default:
		LogError("Unsupported trigger source (not CH1, CH2, or EXT)\n");
		return;
	}

	// Slope
	switch (et->GetType())
	{
	case EdgeTrigger::EDGE_FALLING:
		fpVal |= (FP_TRIG_SLOPE_FALLING << FP_TRIG_SLOPE);
		break;
	case EdgeTrigger::EDGE_RISING:
		fpVal |= (FP_TRIG_SLOPE_RISING << FP_TRIG_SLOPE);
		break;
	default:
		LogError("Unsupported edge trigger type (not rising or falling)\n");
		return;
	}

	// Update FP with new configuration
	AdjustFP(FP_ACQ, FP_TRIG, fpVal, fpMask);

	// Level
	double pos = et->GetLevel() / levelScale;
	SetDAC(levelDac, round(pos * 4095.f) + 4095);
}

void Tektronix22xOscilloscope::PullTrigger()
{
	uint8_t trg = (QueryFP(FP_ACQ) >> FP_TRIG) & 0xff;

	// Clear out any triggers of the wrong type
	if ((m_trigger != NULL) && (dynamic_cast<EdgeTrigger*>(m_trigger) != NULL))
	{
		delete m_trigger;
		m_trigger = NULL;
	}

	// Create a new trigger if necessary
	if (m_trigger == NULL)
		m_trigger = new EdgeTrigger(this);
	EdgeTrigger* et = dynamic_cast<EdgeTrigger*>(m_trigger);

	// Channel parameters
	InstrumentChannel* src = nullptr;
	optional<DAC> levelDac;
	double levelScale = 0.0;
	switch ((trg >> FP_TRIG_SRC) & 0b11)
	{
	case FP_TRIG_SRC_VERT:
		break;
	case FP_TRIG_SRC_CH1:
		src = GetChannel(0);
		levelScale = GetChannelVoltageRange(src->GetIndex(), 0) / NUM_VERTICAL_DIVS * DAC_CH_TRIG_SCALE_DIV;
		levelDac = DAC_CH1_TRIG_LEVEL;
		break;
	case FP_TRIG_SRC_CH2:
		src = GetChannel(1);
		levelScale = GetChannelVoltageRange(src->GetIndex(), 0) / NUM_VERTICAL_DIVS * DAC_CH_TRIG_SCALE_DIV;
		levelDac = DAC_CH2_TRIG_LEVEL;
		break;
	case FP_TRIG_SRC_EXT:
		src = m_extTrigChannel;
		levelScale = DAC_EXT_TRIG_SCALE_V;
		levelDac = DAC_EXT_TRIG_LEVEL;
		break;
	}

	// Source
	if (src)
		et->SetInput(0, StreamDescriptor(src, 0), true);

	// Level
	if (levelDac)
	{
		double pos = (QueryDAC(*levelDac) - 4095) / 4095.f;
		et->SetLevel(pos * levelScale);
	}

	// Slope
	switch ((trg >> FP_TRIG_SLOPE) & 1)
	{
	case FP_TRIG_SLOPE_FALLING:
		et->SetType(EdgeTrigger::EDGE_FALLING);
		break;
	case FP_TRIG_SLOPE_RISING:
		et->SetType(EdgeTrigger::EDGE_RISING);
		break;
	}
}

uint64_t Tektronix22xOscilloscope::GetSampleDepth()
{
	return 512;
}

void Tektronix22xOscilloscope::SetSampleDepth(uint64_t depth)
{
	// Not configurable
	if (depth != GetSampleDepth())
	{
		LogError("Invalid memory depth: %" PRIu64 "\n", depth);
	}
}

vector<uint64_t> Tektronix22xOscilloscope::GetSampleDepthsNonInterleaved()
{
	return {GetSampleDepth()};
}

vector<uint64_t> Tektronix22xOscilloscope::GetSampleDepthsInterleaved()
{
	// Interleaving not supported
	return {};
}

uint64_t Tektronix22xOscilloscope::GetSampleRate()
{
	return GetSampleDepth() / (QueryHorizontalScale() * 1e-9 * NUM_HORIZONTAL_DIVS);
}

void Tektronix22xOscilloscope::SetSampleRate(uint64_t rate)
{
	uint64_t div = GetSampleDepth() / (rate / (1e-9 * NUM_HORIZONTAL_DIVS));
	SetHorizontalScale(div);
}

vector<uint64_t> Tektronix22xOscilloscope::GetSampleRatesNonInterleaved()
{
	vector<uint64_t> rates;
	for (auto scale : FP_HOR_SCALES)
	{
		rates.push_back(GetSampleDepth() / (scale * 1e-9 * NUM_HORIZONTAL_DIVS));
	}
	return rates;
}

vector<uint64_t> Tektronix22xOscilloscope::GetSampleRatesInterleaved()
{
	// Interleaving not supported
	return {};
}

set<Oscilloscope::InterleaveConflict> Tektronix22xOscilloscope::GetInterleaveConflicts()
{
	// Interleaving not supported
	return {};
}

bool Tektronix22xOscilloscope::HasInterleavingControls()
{
	return false;
}

bool Tektronix22xOscilloscope::IsInterleaving()
{
	// Interleaving not supported
	return false;
}

bool Tektronix22xOscilloscope::SetInterleaving([[maybe_unused]] bool combine)
{
	// Interleaving not supported
	return false;
}

bool Tektronix22xOscilloscope::IsSamplingModeAvailable(SamplingMode mode)
{
	// Not configurable
	return GetSamplingMode() == mode;
}

Oscilloscope::SamplingMode Tektronix22xOscilloscope::GetSamplingMode()
{
	// 222: Tektronix document 070-7100-00, page 6-7, table 6-2
	// 224: Tektronix document 070-8476-00, page 3-28, table 3-2
	auto scale = QueryHorizontalScale();
	return (scale <= 2000) ? EQUIVALENT_TIME : REAL_TIME;
}

int64_t Tektronix22xOscilloscope::GetTriggerOffset()
{
	uint64_t fp = QueryFP(FP_ACQ);
	uint8_t miscFp = (fp >> FP_MISC) & 0xff;
	uint8_t trigFp = (fp >> FP_TRIG) & 0xff;
	uint64_t traceLength = static_cast<uint64_t>(QueryHorizontalScale() * FS_PER_NANOSECOND * NUM_HORIZONTAL_DIVS);

	if ((miscFp >> FP_MISC_STORE) & 1)
	{
		switch ((trigFp >> FP_TRIG_POS) & 0b11)
		{
		case FP_TRIG_POS_POST:
		default:
			return 0;
		case FP_TRIG_POS_MID:
			return traceLength / 2;
		case FP_TRIG_POS_PRE:
			return traceLength;
		}
	}
	else
	{
		// 222: Tektronix document 070-7100-00, page 6-12:
		// "The trigger point for a waveform displayed in NONSTORE mode occurs at the sixth data point."
		// 224: Tektronix document 070-8476-00, page 3-70:
		// "In nonstore mode, the trigger is always the sixth sample."
		return static_cast<uint64_t>(traceLength * 6.0 / GetSampleDepth());
	}
}

void Tektronix22xOscilloscope::SetTriggerOffset(int64_t offset)
{
	int64_t traceLength = static_cast<int64_t>(QueryHorizontalScale() * FS_PER_NANOSECOND * NUM_HORIZONTAL_DIVS);

	// We can't be exact, so clamp the first 25% to the start, the mid 50% to the middle,
	// and the last 25% to the end.
	uint8_t point;
	if (offset < traceLength / 4)
		point = FP_TRIG_POS_POST;
	else if (offset < 3 * traceLength / 4)
		point = FP_TRIG_POS_MID;
	else
		point = FP_TRIG_POS_PRE;
	AdjustFP(FP_ACQ, FP_TRIG, point << FP_TRIG_POS, 0b11 << FP_TRIG_POS);
}

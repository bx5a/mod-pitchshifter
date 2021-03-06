#include <stdlib.h>
#include <cmath>
#include "PitchShifterClasses.h"
#include "GainClass.h"
#include "../Freezer/freeze.h"

#include <queue>

/**********************************************************************************************************************************************************/

#define PLUGIN_URI "http://romain-hennequin.fr/plugins/mod-devel/Freeze"
enum {IN, OUT, STEP, CLEAN, PLUGIN_PORT_COUNT};

/**********************************************************************************************************************************************************/

class Freeze
{
public:
    Freeze(uint32_t n_samples, int nBuffers, double samplerate, const std::string& wfile)
    {
        wisdomFile = wfile;
        Construct(n_samples, nBuffers, samplerate, wfile.c_str());
    }
    ~Freeze(){Destruct();}
    void Construct(uint32_t n_samples, int nBuffers, double samplerate, const char* wisdomFile)
    {
    	this->nBuffers = nBuffers;
        SampleRate = samplerate;

        freezer = new freeze::Freezer();
        freezer->Init(1);

        const size_t kBufferLen = 256;
        temp_buffer.resize(kBufferLen);

        dry_gain = 1;

        cont = 0;
    }
    void Destruct()
    {
        delete freezer;
    }
    void Realloc(uint32_t n_samples, int nBuffers)
    {
    	Destruct();
    	Construct(n_samples, nBuffers, SampleRate, wisdomFile.c_str());
    }


    static LV2_Handle instantiate(const LV2_Descriptor* descriptor, double samplerate, const char* bundle_path, const LV2_Feature* const* features);
    static void activate(LV2_Handle instance);
    static void deactivate(LV2_Handle instance);
    static void connect_port(LV2_Handle instance, uint32_t port, void *data);
    static void run(LV2_Handle instance, uint32_t n_samples);
    static void cleanup(LV2_Handle instance);
    static const void* extension_data(const char* uri);
    float *ports[PLUGIN_PORT_COUNT];

    freeze::Freezer* freezer;
    std::queue<float> input_queue, output_queue;
    std::vector<float> temp_buffer;
    float dry_gain;

    int nBuffers;
    int cont;
    double SampleRate;
    std::string wisdomFile;
};

/**********************************************************************************************************************************************************/

static const LV2_Descriptor Descriptor = {
    PLUGIN_URI,
    Freeze::instantiate,
    Freeze::connect_port,
    Freeze::activate,
    Freeze::run,
    Freeze::deactivate,
    Freeze::cleanup,
    Freeze::extension_data
};

/**********************************************************************************************************************************************************/

LV2_SYMBOL_EXPORT
const LV2_Descriptor* lv2_descriptor(uint32_t index)
{
    if (index == 0) return &Descriptor;
    else return NULL;
}

/**********************************************************************************************************************************************************/

LV2_Handle Freeze::instantiate(const LV2_Descriptor* descriptor, double samplerate, const char* bundle_path, const LV2_Feature* const* features)
{
    std::string wisdomFile = bundle_path;
    wisdomFile += "/harmonizer.wisdom";
    const uint32_t n_samples = GetBufferSize(features);
    Freeze *plugin = new Freeze(n_samples, n_samples, samplerate, wisdomFile);
    return (LV2_Handle)plugin;
}

/**********************************************************************************************************************************************************/

void Freeze::activate(LV2_Handle instance){}

/**********************************************************************************************************************************************************/

void Freeze::deactivate(LV2_Handle instance){}

/**********************************************************************************************************************************************************/

void Freeze::connect_port(LV2_Handle instance, uint32_t port, void *data)
{
    Freeze *plugin;
    plugin = (Freeze *) instance;
    plugin->ports[port] = (float*) data;
}

/**********************************************************************************************************************************************************/

void Freeze::run(LV2_Handle instance, uint32_t n_samples)
{
    Freeze *plugin;
    plugin = (Freeze *) instance;

    float *in       = plugin->ports[IN];
    float *out      = plugin->ports[OUT];
    float    step        = (float)(*(plugin->ports[STEP]));
    int c = 0;
    if (step>0.5f)
        c = 1;

    // enable / disable on TOGGLE CLEAN button
    bool enabled = c==1;
    if (enabled && !plugin->freezer->IsEnabled()) {
      plugin->freezer->Enable();
    }
    if (!enabled && plugin->freezer->IsEnabled()) {
      plugin->freezer->Disable();
    }

    // Dry gain factor
    if (plugin->freezer->IsEnabled()) {
      plugin->dry_gain *= 0.8;
    } else {
      plugin->dry_gain = 1.0 - (1.0 - plugin->dry_gain) * 0.8;
    }

    // queue input data
    for (int sample_idx=0; sample_idx < n_samples; sample_idx++) {
      plugin->input_queue.push(in[sample_idx]);
    }

    // dequeue as much data as possible
    std::error_code err;
    while (plugin->input_queue.size() > plugin->temp_buffer.size()) {
      // Get data from input queue
      for (int sample_idx = 0; sample_idx < plugin->temp_buffer.size(); sample_idx++) {
        plugin->temp_buffer[sample_idx] = plugin->input_queue.front();
        plugin->input_queue.pop();
      }

      // write to freezer
      plugin->freezer->Write(plugin->temp_buffer, err);
      if (err) {
        std::cout << "WARNING: Error while writing to freezer: " << err.message() << std::endl;
      }

      // read from freezer
      std::vector<float> result = plugin->freezer->Read(err);
      if (err) {
        std::cout << "WARNING: Error while reading from freezer: " << err.message() << std::endl;
      }

      // Push data to output queue
      for (int sample_idx = 0; sample_idx < result.size(); sample_idx++) {
        plugin->output_queue.push(result[sample_idx] + plugin->dry_gain * plugin->temp_buffer[sample_idx]);
      }
    }

    // Fill output buffer
    for (int sample_idx=0; sample_idx < n_samples; sample_idx++) {
      // Zeros if we don't have enough data available
      if (plugin->output_queue.empty()) {
        out[sample_idx] = 0;
        continue;
      }
      out[sample_idx] = plugin->output_queue.front();
      plugin->output_queue.pop();
    }
}

/**********************************************************************************************************************************************************/

void Freeze::cleanup(LV2_Handle instance)
{
    delete ((Freeze *) instance);
}

/**********************************************************************************************************************************************************/

const void* Freeze::extension_data(const char* uri)
{
    return NULL;
}

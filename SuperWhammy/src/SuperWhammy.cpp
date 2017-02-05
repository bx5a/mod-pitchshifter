#include <stdlib.h>
#include <cmath>
#include "PitchShifterClasses.h"
#include "GainClass.h"
#include "../Freezer/freeze.h"

#include <queue>

/**********************************************************************************************************************************************************/

#define PLUGIN_URI "http://moddevices.com/plugins/mod-devel/SuperWhammy"
#define FIDELITY0 6,3,2,1
#define FIDELITY1 12,6,3,2
#define FIDELITY2 16,8,4,2
#define FIDELITY3 20,10,5,3
enum {IN, OUT, STEP, FIRST, LAST, CLEAN, GAIN, FIDELITY, PLUGIN_PORT_COUNT};

/**********************************************************************************************************************************************************/

class SuperWhammy
{
public:
    SuperWhammy(uint32_t n_samples, int nBuffers, double samplerate, const std::string& wfile)
    {
        wisdomFile = wfile;
        Construct(n_samples, nBuffers, samplerate, wfile.c_str());
    }
    ~SuperWhammy(){Destruct();}
    void Construct(uint32_t n_samples, int nBuffers, double samplerate, const char* wisdomFile)
    {
    	this->nBuffers = nBuffers;
        SampleRate = samplerate;

        obja = new PSAnalysis(n_samples, nBuffers, wisdomFile);
        objs = new PSSinthesis(obja, wisdomFile);
        objg = new GainClass(n_samples);
        freezer = new freeze::Freezer();
        freezer->Init(1);
      
        const size_t kBufferLen = 256;
        temp_buffer.resize(kBufferLen);
      
        dry_gain = 1;

        cont = 0;
    }
    void Destruct()
    {
        delete obja;
        delete objs;
        delete objg;
        delete freezer;
    }
    void Realloc(uint32_t n_samples, int nBuffers)
    {
    	Destruct();
    	Construct(n_samples, nBuffers, SampleRate, wisdomFile.c_str());
    }

    void SetFidelity(int fidelity, uint32_t n_samples)
    {
        int bufsize;

        switch (fidelity)
        {
        case 0: 
            bufsize = nBuffersSW(n_samples,FIDELITY0);
            break;
        case 1:
            bufsize = nBuffersSW(n_samples,FIDELITY1);
            break;
        case 2:
            bufsize = nBuffersSW(n_samples,FIDELITY2);
            break;
        case 3:
            bufsize = nBuffersSW(n_samples,FIDELITY3);
            break;
        default:
            return;
        }

        if (nBuffers != bufsize || obja->hopa != (int)n_samples)
            Realloc(n_samples, bufsize);
    }

    static LV2_Handle instantiate(const LV2_Descriptor* descriptor, double samplerate, const char* bundle_path, const LV2_Feature* const* features);
    static void activate(LV2_Handle instance);
    static void deactivate(LV2_Handle instance);
    static void connect_port(LV2_Handle instance, uint32_t port, void *data);
    static void run(LV2_Handle instance, uint32_t n_samples);
    static void cleanup(LV2_Handle instance);
    static const void* extension_data(const char* uri);
    float *ports[PLUGIN_PORT_COUNT];
    
    PSAnalysis *obja;
    PSSinthesis *objs;
    GainClass *objg;
  
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
    SuperWhammy::instantiate,
    SuperWhammy::connect_port,
    SuperWhammy::activate,
    SuperWhammy::run,
    SuperWhammy::deactivate,
    SuperWhammy::cleanup,
    SuperWhammy::extension_data
};

/**********************************************************************************************************************************************************/

LV2_SYMBOL_EXPORT
const LV2_Descriptor* lv2_descriptor(uint32_t index)
{
    if (index == 0) return &Descriptor;
    else return NULL;
}

/**********************************************************************************************************************************************************/

LV2_Handle SuperWhammy::instantiate(const LV2_Descriptor* descriptor, double samplerate, const char* bundle_path, const LV2_Feature* const* features)
{
    std::string wisdomFile = bundle_path;
    wisdomFile += "/harmonizer.wisdom";
    const uint32_t n_samples = GetBufferSize(features);
    SuperWhammy *plugin = new SuperWhammy(n_samples, nBuffersSW(n_samples,FIDELITY1), samplerate, wisdomFile);
    return (LV2_Handle)plugin;
}

/**********************************************************************************************************************************************************/

void SuperWhammy::activate(LV2_Handle instance){}

/**********************************************************************************************************************************************************/

void SuperWhammy::deactivate(LV2_Handle instance){}

/**********************************************************************************************************************************************************/

void SuperWhammy::connect_port(LV2_Handle instance, uint32_t port, void *data)
{
    SuperWhammy *plugin;
    plugin = (SuperWhammy *) instance;
    plugin->ports[port] = (float*) data;
}

/**********************************************************************************************************************************************************/

void SuperWhammy::run(LV2_Handle instance, uint32_t n_samples)
{
    SuperWhammy *plugin;
    plugin = (SuperWhammy *) instance;

    float *in       = plugin->ports[IN];
    float *out      = plugin->ports[OUT];
    int    c        = (int)(*(plugin->ports[CLEAN])+0.5f);
  
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

void SuperWhammy::cleanup(LV2_Handle instance)
{
    delete ((SuperWhammy *) instance);
}

/**********************************************************************************************************************************************************/

const void* SuperWhammy::extension_data(const char* uri)
{
    return NULL;
}

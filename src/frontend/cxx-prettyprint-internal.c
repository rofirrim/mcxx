#include "cxx-prettyprint-internal.h"

// Initial behaviour
prettyprint_behaviour_t prettyprint_behaviour = 
{ 
    /* .internal_output = */ 1 
};

void prettyprint_set_not_internal_output(void)
{
    prettyprint_behaviour.internal_output = 0;
}

void prettyprint_set_internal_output(void)
{
    prettyprint_behaviour.internal_output = 1;
}

void prettyprint_context_init(prettyprint_context_t* pt_ctx)
{
    memset(pt_ctx, 0, sizeof(*pt_ctx));
    // This will be configurable one day
    pt_ctx->indent_str = "    ";
    pt_ctx->level = 0;
    pt_ctx->internal_output = prettyprint_behaviour.internal_output;
}

void prettyprint_context_copy(prettyprint_context_t* dest,
        const prettyprint_context_t* src)
{
    // Nothing else must be done
    memcpy(dest, src, sizeof(*dest));
}

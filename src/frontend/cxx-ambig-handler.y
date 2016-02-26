/*!if GRAMMAR_CODE*/
#include "cxx-utils.h"

static AST ambiguityHandler (YYSTYPE x0, YYSTYPE x1)
{
	AST son0 = x0.ast;
	AST son1 = x1.ast;

	if (son0 == son1) 
	{
        return son1;
	}

if (ast_get_kind(son0) == AST_AMBIGUITY)
{
int i, n = ast_get_num_ambiguities(son0);
for (i = 0; i < n; i++)
{
   ERROR_CONDITION(ast_get_ambiguity(son0, i) == son1, "Repeated ambiguity", 0);
}
}

if (ast_get_kind(son1) == AST_AMBIGUITY)
{
int i, n = ast_get_num_ambiguities(son1);
for (i = 0; i < n; i++)
{
   ERROR_CONDITION(ast_get_ambiguity(son1, i) == son0, "Repeated ambiguity", 0);
}
}

    return ast_make_ambiguous(son0, son1);
}
/*!endif*/

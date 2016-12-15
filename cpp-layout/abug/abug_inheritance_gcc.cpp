struct empty {};

struct non_pod_in_cpp98 : public empty {};

void should_not_compile()
{
    goto label;
    non_pod_in_cpp98 x;
label:
    (void)x;
}


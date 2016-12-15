struct non_pod_in_cpp98
{
protected:
    int x;
};

void should_not_compile()
{
    goto label;
    non_pod_in_cpp98 x;
label:
    (void)x;
}


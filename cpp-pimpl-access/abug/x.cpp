struct Outer
{
    struct Inner
    {
        friend int get(Outer& o)
        {
            return o.x;
        }
    };
private:
    int x;
};

Elf dependencies in example look like this:

      .------.
      | main |-------.
      '------'       |
       elf|       elf|
       sym|          |
          v          |
      .------.       |
      | trs  |       |
      '------'       |
          |          v
       elf|  sym .------.
       sym| .--->| osc  |
          | |    '------'
          v |        |
      .------.  elf  |
      | uk   |<------'
      '------'

There are two dependencies between uk and osc, forming a cycle:
1. libuk.so needs one symbol from libosc.so
2. libosc.so elf file has explicit dependency on libuk.so

Linking executable with --as-needed is impossible, since --as-needed removes libosc.so from the picture completely.
Removing libosc.so from a.out is valid, since it's not needed directly in final a.out.
Since libosc.so is not an a.out dependency and doesn't exist in dependency hierarchy of other libraries, we have a missing symbol.

